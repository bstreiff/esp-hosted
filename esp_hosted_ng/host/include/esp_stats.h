// SPDX-License-Identifier: GPL-2.0-only
/*
 * Espressif Systems Wireless LAN device driver
 *
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 */
#ifndef __ESP_STAT__H__
#define __ESP_STAT__H__

#include "esp.h"

#if TEST_RAW_TP

#define TEST_RAW_TP__BUF_SIZE    1460

#define ESP_TEST_RAW_TP__RX      0
#define ESP_TEST_RAW_TP__TX      1

void esp_raw_tp_queue_resume(struct esp_adapter *adapter);
#endif

void test_raw_tp_cleanup(struct esp_adapter *adapter);
void update_test_raw_tp_rx_stats(struct esp_adapter *adapter, u16 len);

#endif
