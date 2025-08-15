// SPDX-License-Identifier: GPL-2.0-only
/*
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 */
#include "utils.h"
#include <linux/spi/spi.h>
#include <linux/gpio/consumer.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of.h>
#include "esp_spi.h"
#include "esp_if.h"
#include "esp_api.h"
#include "esp_bt_api.h"
#include "esp_kernel_port.h"
#include "esp_stats.h"
#include "esp_utils.h"
#include "esp_cfg80211.h"

#define SPI_INITIAL_CLK_MHZ     10
#define TX_MAX_PENDING_COUNT    100
#define TX_RESUME_THRESHOLD     (TX_MAX_PENDING_COUNT/5)

extern u32 raw_tp_mode;

static struct sk_buff *read_packet(struct esp_adapter *adapter);
static int write_packet(struct esp_adapter *adapter, struct sk_buff *skb);
static void spi_exit(struct esp_spi_context *context);
static int spi_init(struct spi_device *spi, struct esp_spi_context *context);
static int adjust_spi_clock(struct esp_adapter *adapter, u8 spi_clk_mhz);
static int validate_chipset(struct esp_adapter *adapter, u8 chipset);
static int tx_reset(struct esp_adapter *adapter);

static struct esp_if_ops if_ops = {
	.read		= read_packet,
	.write		= write_packet,
	.adjust_spi_clock = adjust_spi_clock,
	.validate_chipset = validate_chipset,
	.tx_reset = tx_reset,
};

static DEFINE_MUTEX(spi_lock);

static void open_data_path(struct esp_spi_context *context)
{
	atomic_set(&context->tx_pending, 0);
	msleep(200);
	set_bit(ESP_SPI_DATAPATH_OPEN, &context->spi_flags);
}

static void close_data_path(struct esp_spi_context *context)
{
	clear_bit(ESP_SPI_DATAPATH_OPEN, &context->spi_flags);
	msleep(200);
}

static irqreturn_t spi_data_ready_interrupt_handler(int irq, void *cookie)
{
	struct esp_spi_context *context = (struct esp_spi_context *) cookie;

	/* ESP peripheral has queued buffer for transmission */
	if (context->spi_workqueue)
		queue_work(context->spi_workqueue, &context->spi_work);

	return IRQ_HANDLED;
 }

static irqreturn_t spi_interrupt_handler(int irq, void *cookie)
{
	struct esp_spi_context *context = (struct esp_spi_context *) cookie;

	/* ESP peripheral is ready for next SPI transaction */
	if (context->spi_workqueue)
		queue_work(context->spi_workqueue, &context->spi_work);

	return IRQ_HANDLED;
}

static struct sk_buff *read_packet(struct esp_adapter *adapter)
{
	struct esp_spi_context *context;
	struct sk_buff *skb = NULL;

	if (!adapter || !adapter->if_context) {
		esp_err("Invalid args\n");
		return NULL;
	}

	context = adapter->if_context;

	if (!test_bit(ESP_SPI_DATAPATH_OPEN, &context->spi_flags)) {
		return NULL;
	}

	if (context->esp_spi_dev) {
		skb = skb_dequeue(&(context->rx_q[PRIO_Q_HIGH]));
		if (!skb)
			skb = skb_dequeue(&(context->rx_q[PRIO_Q_MID]));
		if (!skb)
			skb = skb_dequeue(&(context->rx_q[PRIO_Q_LOW]));
	} else {
		esp_err("Invalid args\n");
		return NULL;
	}

	return skb;
}

static int write_packet(struct esp_adapter *adapter, struct sk_buff *skb)
{
	struct esp_spi_context *context = (struct esp_spi_context *) adapter->if_context;
	u32 max_pkt_size = SPI_BUF_SIZE - sizeof(struct esp_payload_header);
	struct esp_payload_header *payload_header = (struct esp_payload_header *) skb->data;
	struct esp_skb_cb *cb = NULL;

	if (!adapter || !adapter->if_context || !skb || !skb->data || !skb->len) {
		esp_err("Invalid args\n");
		if (skb) {
			dev_kfree_skb(skb);
			skb = NULL;
		}
		return -EINVAL;
	}

	if (skb->len > max_pkt_size) {
		esp_err("Drop pkt of len[%u] > max spi transport len[%u]\n",
				skb->len, max_pkt_size);
		dev_kfree_skb(skb);
		return -EPERM;
	}

	if (!test_bit(ESP_SPI_DATAPATH_OPEN, &context->spi_flags)) {
		esp_info("%u datapath closed\n", __LINE__);
		dev_kfree_skb(skb);
		return -EPERM;
	}

	cb = (struct esp_skb_cb *)skb->cb;
	if (cb && cb->priv && (atomic_read(&context->tx_pending) >= TX_MAX_PENDING_COUNT)) {
		esp_tx_pause(cb->priv);
		dev_kfree_skb(skb);
		skb = NULL;
		esp_verbose("TX Pause busy");
		if (context->spi_workqueue)
			queue_work(context->spi_workqueue, &context->spi_work);
		return -EBUSY;
	}

	/* Enqueue SKB in tx_q */
	if (payload_header->if_type == ESP_INTERNAL_IF) {
		skb_queue_tail(&context->tx_q[PRIO_Q_HIGH], skb);
	} else if (payload_header->if_type == ESP_HCI_IF) {
		skb_queue_tail(&context->tx_q[PRIO_Q_MID], skb);
	} else {
		skb_queue_tail(&context->tx_q[PRIO_Q_LOW], skb);
		atomic_inc(&context->tx_pending);
	}

	if (context->spi_workqueue)
		queue_work(context->spi_workqueue, &context->spi_work);

	return 0;
}

static int validate_chipset(struct esp_adapter *adapter, u8 chipset)
{
	int ret = 0;

	switch(chipset) {
	case ESP_FIRMWARE_CHIP_ESP32:
	case ESP_FIRMWARE_CHIP_ESP32S2:
	case ESP_FIRMWARE_CHIP_ESP32S3:
	case ESP_FIRMWARE_CHIP_ESP32C2:
	case ESP_FIRMWARE_CHIP_ESP32C3:
	case ESP_FIRMWARE_CHIP_ESP32C6:
	case ESP_FIRMWARE_CHIP_ESP32C5:
		adapter->chipset = chipset;
		esp_info("Chipset=%s ID=%02x detected over SPI\n", esp_chipname_from_id(chipset), chipset);
		break;
	default:
		esp_err("Unrecognized chipset ID=%02x\n", chipset);
		adapter->chipset = ESP_FIRMWARE_CHIP_UNRECOGNIZED;
		break;
	}

	return ret;
}

static int tx_reset(struct esp_adapter *adapter)
{
	struct esp_spi_context *context = (struct esp_spi_context *) adapter->if_context;

	/* Second & onward bootup cleanup:
	 *
	 * SPI is software and not a hardware based module.
	 * When bootup event is received, we should discard all prior commands,
	 * old messages pending at network and re-initialize everything.
	 */
	uint8_t prio_q_idx, iface_idx;

	for (prio_q_idx = 0; prio_q_idx < MAX_PRIORITY_QUEUES; prio_q_idx++) {
		skb_queue_purge(&context->tx_q[prio_q_idx]);
	}

	for (iface_idx = 0; iface_idx < ESP_MAX_INTERFACE; iface_idx++) {
		struct esp_wifi_device *priv = adapter->priv[iface_idx];
		esp_mark_scan_done_and_disconnect(priv, true);
	}

	esp_remove_card(adapter);

	for (prio_q_idx = 0; prio_q_idx < MAX_PRIORITY_QUEUES; prio_q_idx++) {
		skb_queue_head_init(&context->tx_q[prio_q_idx]);
	}

	return 0;
}

static int process_rx_buf(struct esp_spi_context *context, struct sk_buff *skb)
{
	struct esp_payload_header *header;
	u16 len = 0;
	u16 offset = 0;

	if (!skb)
		return -EINVAL;

	header = (struct esp_payload_header *) skb->data;

	if (header->if_type >= ESP_MAX_IF) {
		return -EINVAL;
	}

	offset = le16_to_cpu(header->offset);

	/* Validate received SKB. Check len and offset fields */
	if (offset != sizeof(struct esp_payload_header)) {
		esp_info("offset_rcv[%d] != exp[%d], drop\n",
				(int)offset, (int)sizeof(struct esp_payload_header));
		return -EINVAL;
	}

	len = le16_to_cpu(header->len);
	if (!len) {
		return -EINVAL;
	}

	len += sizeof(struct esp_payload_header);

	if (len > SPI_BUF_SIZE) {
		return -EINVAL;
	}

	/* Trim SKB to actual size */
	skb_trim(skb, len);


	if (!test_bit(ESP_SPI_DATAPATH_OPEN, &context->spi_flags)) {
		esp_verbose("%u datapath closed\n", __LINE__);
		return -EPERM;
	}

	/* enqueue skb for read_packet to pick it */
	if (header->if_type == ESP_INTERNAL_IF)
		skb_queue_tail(&context->rx_q[PRIO_Q_HIGH], skb);
	else if (header->if_type == ESP_HCI_IF)
		skb_queue_tail(&context->rx_q[PRIO_Q_MID], skb);
	else
		skb_queue_tail(&context->rx_q[PRIO_Q_LOW], skb);

	/* indicate reception of new packet */
	esp_process_new_packet_intr(context->adapter);

	return 0;
}

static void esp_spi_work(struct work_struct *work)
{
	struct esp_spi_context *context = container_of(work, struct esp_spi_context, spi_work);
	struct spi_transfer trans;
	struct sk_buff *tx_skb = NULL, *rx_skb = NULL;
	struct esp_skb_cb *cb = NULL;
	u8 *rx_buf = NULL;
	int ret = 0;
	volatile int trans_ready, rx_pending;

	mutex_lock(&spi_lock);

	trans_ready = gpiod_get_value(context->handshake_gpio);
	rx_pending = gpiod_get_value(context->dataready_gpio);

	if (trans_ready) {
		if (test_bit(ESP_SPI_DATAPATH_OPEN, &context->spi_flags)) {
			tx_skb = skb_dequeue(&context->tx_q[PRIO_Q_HIGH]);
			if (!tx_skb)
				tx_skb = skb_dequeue(&context->tx_q[PRIO_Q_MID]);
			if (!tx_skb)
				tx_skb = skb_dequeue(&context->tx_q[PRIO_Q_LOW]);
			if (tx_skb) {
				if (atomic_read(&context->tx_pending))
					atomic_dec(&context->tx_pending);

				/* resume network tx queue if bearable load */
				cb = (struct esp_skb_cb *)tx_skb->cb;
				if (cb && cb->priv && atomic_read(&context->tx_pending) < TX_RESUME_THRESHOLD) {
					esp_tx_resume(cb->priv);
#if TEST_RAW_TP
					if (raw_tp_mode != 0) {
						esp_raw_tp_queue_resume(context->adapter);
					}
#endif
				}
			}
		}

		if (rx_pending || tx_skb) {
			memset(&trans, 0, sizeof(trans));
			trans.speed_hz = context->speed_hz;

			/* Setup and execute SPI transaction
			 *	Tx_buf: Check if tx_q has valid buffer for transmission,
			 *		else keep it blank
			 *
			 *	Rx_buf: Allocate memory for incoming data. This will be freed
			 *		immediately if received buffer is invalid.
			 *		If it is a valid buffer, upper layer will free it.
			 * */

			/* Configure TX buffer if available */

			if (tx_skb) {
				trans.tx_buf = tx_skb->data;
				esp_hex_dump_verbose("tx: ", trans.tx_buf, 32);
			} else {
				tx_skb = esp_alloc_skb(SPI_BUF_SIZE);
				trans.tx_buf = skb_put(tx_skb, SPI_BUF_SIZE);
				memset((void *)trans.tx_buf, 0, SPI_BUF_SIZE);
			}

			/* Configure RX buffer */
			rx_skb = esp_alloc_skb(SPI_BUF_SIZE);
			rx_buf = skb_put(rx_skb, SPI_BUF_SIZE);

			memset(rx_buf, 0, SPI_BUF_SIZE);

			trans.rx_buf = rx_buf;
			trans.len = SPI_BUF_SIZE;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 15, 0))
			if (context->adapter->chipset == ESP_FIRMWARE_CHIP_ESP32) {
				trans.cs_change = 1;
			}
#endif

			ret = spi_sync_transfer(context->esp_spi_dev, &trans, 1);
			if (ret) {
				esp_err("SPI Transaction failed: %d", ret);
				dev_kfree_skb(rx_skb);
				dev_kfree_skb(tx_skb);
			} else {

				/* Free rx_skb if received data is not valid */
				if (process_rx_buf(context, rx_skb)) {
					dev_kfree_skb(rx_skb);
				}

				if (tx_skb)
					dev_kfree_skb(tx_skb);
			}
		}
	}

	mutex_unlock(&spi_lock);
}

static int spi_dev_init(struct spi_device *spi, struct esp_spi_context *context)
{
	int status = 0;

	spi_set_drvdata(spi, context);

	set_bit(ESP_SPI_BUS_CLAIMED, &context->spi_flags);
	context->esp_spi_dev = spi;

	status = spi_setup(context->esp_spi_dev);

	if (status) {
		esp_err("Failed to setup new SPI device");
		return status;
	}

	esp_info("Config - SPI clock[%u Hz] bus[%d] cs[%d] mode[%d]\n",
		context->speed_hz, spi->controller->bus_num,
		(int)spi->chip_select, spi->mode);

	set_bit(ESP_SPI_BUS_SET, &context->spi_flags);

	context->handshake_gpio = gpiod_get(&spi->dev, "handshake", GPIOD_IN);
	if (IS_ERR(context->handshake_gpio)) {
		status = PTR_ERR(context->handshake_gpio);
		esp_err("Failed to obtain GPIO for Handshake pin, err:%d\n", status);
		return status;
	}

	set_bit(ESP_SPI_GPIO_HS_REQUESTED, &context->spi_flags);

	status = request_irq(gpiod_to_irq(context->handshake_gpio), spi_interrupt_handler,
			IRQF_SHARED | IRQF_TRIGGER_RISING,
			"esp32 handshake", context);
	if (status) {
		esp_err("Failed to request IRQ for Handshake pin, err:%d\n", status);
		return status;
	}
	set_bit(ESP_SPI_GPIO_HS_IRQ_DONE, &context->spi_flags);

	context->dataready_gpio = gpiod_get(&spi->dev, "dataready", GPIOD_IN);
	if (IS_ERR(context->dataready_gpio)) {
		status = PTR_ERR(context->dataready_gpio);
		free_irq(gpiod_to_irq(context->handshake_gpio), context);
		gpiod_put(context->handshake_gpio);
		esp_err("Failed to obtain GPIO for Data ready pin, err:%d\n", status);
		return status;
	}
	set_bit(ESP_SPI_GPIO_DR_REQUESTED, &context->spi_flags);

	status = request_irq(gpiod_to_irq(context->dataready_gpio), spi_data_ready_interrupt_handler,
			IRQF_SHARED | IRQF_TRIGGER_RISING,
			"esp32 dataready", context);
	if (status) {
		gpiod_put(context->dataready_gpio);
		free_irq(gpiod_to_irq(context->handshake_gpio), context);
		gpiod_put(context->handshake_gpio);
		esp_err("Failed to request IRQ for Data ready pin, err:%d\n", status);
		return status;
	}
	set_bit(ESP_SPI_GPIO_DR_IRQ_DONE, &context->spi_flags);

	esp_info("Config - SPI GPIOs: Handshake[%d] Dataready[%d]\n",
		desc_to_gpio(context->handshake_gpio),
		desc_to_gpio(context->dataready_gpio));

	open_data_path(context);

	return 0;
}

static int spi_init(struct spi_device *spi, struct esp_spi_context *context)
{
	int status = 0;
	uint8_t prio_q_idx = 0;
	struct esp_adapter *adapter;

	context->spi_workqueue = create_workqueue("ESP_SPI_WORK_QUEUE");

	if (!context->spi_workqueue) {
		esp_err("spi workqueue failed to create\n");
		spi_exit(context);
		return -EFAULT;
	}

	INIT_WORK(&context->spi_work, esp_spi_work);

	for (prio_q_idx = 0; prio_q_idx < MAX_PRIORITY_QUEUES; prio_q_idx++) {
		skb_queue_head_init(&context->tx_q[prio_q_idx]);
		skb_queue_head_init(&context->rx_q[prio_q_idx]);
	}

	status = spi_dev_init(spi, context);
	if (status) {
		spi_exit(context);
		esp_err("Failed Init SPI device\n");
		return status;
	}

	adapter = context->adapter;
	atomic_set(&adapter->state, ESP_CONTEXT_READY);

	if (!adapter) {
		spi_exit(context);
		return -EFAULT;
	}

	adapter->dev = &context->esp_spi_dev->dev;

	return status;
}

static void cleanup_spi_gpio(struct esp_spi_context *context)
{
	if (test_bit(ESP_SPI_GPIO_HS_IRQ_DONE, &context->spi_flags)) {
		free_irq(gpiod_to_irq(context->handshake_gpio), context);
		clear_bit(ESP_SPI_GPIO_HS_IRQ_DONE, &context->spi_flags);
	}

	if (test_bit(ESP_SPI_GPIO_DR_IRQ_DONE, &context->spi_flags)) {
		free_irq(gpiod_to_irq(context->dataready_gpio), context);
		clear_bit(ESP_SPI_GPIO_DR_IRQ_DONE, &context->spi_flags);
	}

	if (test_bit(ESP_SPI_GPIO_DR_REQUESTED, &context->spi_flags)) {
		gpiod_put(context->dataready_gpio);
		clear_bit(ESP_SPI_GPIO_DR_REQUESTED, &context->spi_flags);
	}

	if (test_bit(ESP_SPI_GPIO_HS_REQUESTED, &context->spi_flags)) {
		gpiod_put(context->handshake_gpio);
		clear_bit(ESP_SPI_GPIO_HS_REQUESTED, &context->spi_flags);
	}
}

static void spi_exit(struct esp_spi_context *context)
{
	uint8_t prio_q_idx = 0;
	if (context->adapter)
		atomic_set(&context->adapter->state, ESP_CONTEXT_DISABLED);

	if (test_bit(ESP_SPI_GPIO_HS_IRQ_DONE, &context->spi_flags)) {
		disable_irq(gpiod_to_irq(context->handshake_gpio));
	}

	if (test_bit(ESP_SPI_GPIO_DR_IRQ_DONE, &context->spi_flags)) {
		disable_irq(gpiod_to_irq(context->dataready_gpio));
	}

	close_data_path(context);
	msleep(200);

	for (prio_q_idx = 0; prio_q_idx < MAX_PRIORITY_QUEUES; prio_q_idx++) {
		skb_queue_purge(&context->tx_q[prio_q_idx]);
		skb_queue_purge(&context->rx_q[prio_q_idx]);
	}

	if (context->spi_workqueue) {
		flush_workqueue(context->spi_workqueue);
		destroy_workqueue(context->spi_workqueue);
		context->spi_workqueue = NULL;
	}

	esp_remove_card(context->adapter);

	cleanup_spi_gpio(context);

	if (context->adapter && context->adapter->hcidev)
		esp_deinit_bt(context->adapter);

	context->adapter->dev = NULL;
}

static int adjust_spi_clock(struct esp_adapter *adapter, u8 spi_clk_mhz)
{
	struct esp_spi_context *context = (struct esp_spi_context *) adapter->if_context;
	struct spi_device *spi = context->esp_spi_dev;

	if (spi_clk_mhz && (context->speed_hz != (spi_clk_mhz * NUMBER_1M))) {
		esp_info("ESP Reconfigure SPI CLK to %u MHz\n", spi_clk_mhz);
		context->speed_hz = spi_clk_mhz * NUMBER_1M;
		if (spi->max_speed_hz && (context->speed_hz > spi->max_speed_hz)) {
			esp_info("Limiting to %u Hz\n", spi->max_speed_hz);
			context->speed_hz = spi->max_speed_hz;
		}
	}

	return 0;
}

int generate_slave_intr(void *context, u8 data)
{
	return 0;
}

static int esp_spi_probe(struct spi_device *spi)
{
	struct esp_spi_context *context;
	struct esp_adapter *adapter = esp_adapter_create(&spi->dev);

	if (!adapter) {
		esp_err("unable to create adapter\n");
		return -EFAULT;
	}

	context = kzalloc(sizeof(*context), GFP_KERNEL);
	if (!context) {
		esp_err("unable to create context\n");
		esp_adapter_destroy(adapter);
		return -EFAULT;
	}

	adapter->if_context = context;
	adapter->if_ops = &if_ops;
	adapter->if_type = ESP_IF_TYPE_SPI;
	context->adapter = adapter;
	context->speed_hz = SPI_INITIAL_CLK_MHZ * NUMBER_1M;
	if (spi->max_speed_hz && (context->speed_hz > spi->max_speed_hz))
		context->speed_hz = spi->max_speed_hz;

	return spi_init(spi, context);
}

static void esp_spi_remove(struct spi_device *spi)
{
	struct esp_spi_context *context = spi_get_drvdata(spi);
	struct esp_adapter *adapter = context->adapter;

	if (!adapter || !context)
		return;

	spi_exit(context);

	kfree(adapter->if_context);
	adapter->if_context = NULL;

	memset(&context, 0, sizeof(context));

	esp_adapter_destroy(adapter);
}

static const struct of_device_id esp_spi_of_match[] = {
	{ .compatible = "espressif,esp32", },
	{},
};
MODULE_DEVICE_TABLE(of, esp_spi_of_match);

static const struct spi_device_id esp_spi_ids[] = {
	{ .name = "esp32", },
	{},
};
MODULE_DEVICE_TABLE(spi, esp_spi_ids);

static struct spi_driver esp_spi_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = of_match_ptr(esp_spi_of_match),
	},
	.probe = esp_spi_probe,
	.remove = esp_spi_remove,
	.id_table = esp_spi_ids,
};

int esp_init_interface_layer()
{
	return spi_register_driver(&esp_spi_driver);
}

void esp_deinit_interface_layer()
{
	return spi_unregister_driver(&esp_spi_driver);
}
