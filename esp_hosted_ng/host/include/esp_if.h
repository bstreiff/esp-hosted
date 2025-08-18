// SPDX-License-Identifier: GPL-2.0-only
/*
 * Espressif Systems Wireless LAN device driver
 *
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 */
#ifndef __ESP_IF__H_
#define __ESP_IF__H_

#include "esp.h"

struct esp_if_ops {
	int (*init)(struct esp_adapter *adapter);
	struct sk_buff* (*read)(struct esp_adapter *adapter);
	int (*write)(struct esp_adapter *adapter, struct sk_buff *skb);
	int (*deinit)(struct esp_adapter *adapter);

	int (*adjust_spi_clock)(struct esp_adapter *adapter, u8 spi_clk_mhz);
	int (*validate_chipset)(struct esp_adapter *adapter, u8 chipset);
	int (*tx_reset)(struct esp_adapter *adapter);
	int (*disable_data_path)(struct esp_adapter *adapter);
};

#endif
