/*
 * SPDX-FileCopyrightText: 2026 Antigravity
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Заголовочный файл автономного Zigbee-Координатора для управления реле
 */

#pragma once

#include "esp_err.h"
#include "esp_zigbee.h"
#include "ezbee/zha.h"
#include "ezbee/zcl/cluster/on_off.h"
#include "ezbee/zcl/cluster/basic.h"
#include "ezbee/zcl/cluster/identify.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ESP_ZIGBEE_COORDINATOR_ENDPOINT_1 (1)
#define ESP_ZIGBEE_COORDINATOR_ENDPOINT_2 (2)
#define ESP_ZIGBEE_COORDINATOR_ENDPOINT (1)

#define ESP_MANUFACTURER_NAME "\x09""ESPRESSIF"
#define ESP_MODEL_IDENTIFIER  "\x0E""ESP32C6_COORD"

/* Фиксированный чистый канал Zigbee (Канал 15) для стабильного сопряжения */
#define ESP_ZIGBEE_PRIMARY_CHANNEL_MASK (1U << 15)
#define ESP_ZIGBEE_SECONDARY_CHANNEL_MASK (1U << 15)

#define SWITCH_BUTTON_GPIO GPIO_NUM_9

#define ESP_ZIGBEE_ZC_CONFIG()                          \
    {                                                   \
        .device_type = EZB_NWK_DEVICE_TYPE_COORDINATOR, \
        .install_code_policy = false,                   \
        .zczr_config = {                                \
            .max_children = 10,                         \
        },                                              \
    }

#if CONFIG_SOC_IEEE802154_SUPPORTED
#define ESP_ZIGBEE_PLATFORM_CONFIG()                                 \
    {                                                                \
        .storage_partition_name = "nvs",                             \
        .radio_config = {                                            \
            .radio_mode = ESP_ZIGBEE_RADIO_MODE_NATIVE,              \
        },                                                           \
    }
#endif

#define ESP_ZIGBEE_DEFAULT_CONFIG()                      \
    {                                                    \
        .device_config = ESP_ZIGBEE_ZC_CONFIG(),          \
        .platform_config = ESP_ZIGBEE_PLATFORM_CONFIG(), \
    }

#ifdef __cplusplus
}
#endif
