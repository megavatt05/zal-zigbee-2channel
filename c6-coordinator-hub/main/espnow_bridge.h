/*
 * SPDX-FileCopyrightText: 2026 Antigravity
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ESP-NOW -> Zigbee 3.0 Bridge Header
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ESPNOW_SWITCH_MAGIC 0xC3

typedef enum {
    ESPNOW_ACTION_OFF    = 0,
    ESPNOW_ACTION_ON     = 1,
    ESPNOW_ACTION_TOGGLE = 2,
} espnow_action_t;

/**
 * @brief Пакет команды выключателя ESP32-C3
 */
typedef struct __attribute__((packed)) {
    uint8_t magic;      // 0xC3 (валидация протокола)
    uint8_t line_id;    // 1 = Линия 1, 2 = Линия 2
    uint8_t action;     // 0 = OFF, 1 = ON, 2 = TOGGLE
    uint8_t seq;        // Счетчик нажатий
    uint16_t vdd_mv;    // Напряжение батареи C3 в мВ (например 3000 мВ = 3.0 В)
} espnow_switch_pkt_t;

typedef void (*espnow_bridge_cmd_cb_t)(uint8_t line_id, uint8_t action, uint16_t vdd_mv);

/**
 * @brief Инициализация Wi-Fi и ESP-NOW в режиме приёмника моста
 * @param channel Wi-Fi канал для приёма (рекомендуется 1)
 * @param cmd_cb Callback-функция обработки команд
 */
esp_err_t espnow_bridge_init(uint8_t channel, espnow_bridge_cmd_cb_t cmd_cb);

#ifdef __cplusplus
}
#endif
