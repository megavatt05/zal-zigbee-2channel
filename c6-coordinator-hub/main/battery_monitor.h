/*
 * SPDX-FileCopyrightText: 2026 Antigravity
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Мониторинг напряжения и уровня заряда батареи для Zigbee
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int  adc_channel;       /*!< Номер канала ADC1 (-1 если отключен / фиктивный) */
    int  divider_r1_kohm;   /*!< Верхний резистор делителя (кОм) */
    int  divider_r2_kohm;   /*!< Нижний резистор делителя (кОм) */
    int  min_voltage_mv;    /*!< Минимальное напряжение (0%), например 2600 мВ для CR2032/CR2450 */
    int  max_voltage_mv;    /*!< Максимальное напряжение (100%), например 3000 мВ для CR2032/CR2450 */
} battery_monitor_config_t;

/**
 * @brief Инициализация монитора батареи
 */
esp_err_t battery_monitor_init(const battery_monitor_config_t *config);

/**
 * @brief Чтение текущего напряжения в милливольтах
 */
uint32_t battery_monitor_get_voltage_mv(void);

/**
 * @brief Получение напряжения для Zigbee Power Configuration (единицы 100 мВ, например 30 = 3.0 В)
 */
uint8_t battery_monitor_get_zb_voltage(void);

/**
 * @brief Получение оставшегося заряда в процентах (0..100%)
 */
uint8_t battery_monitor_get_percentage(void);

/**
 * @brief Получение значения для Zigbee Power Configuration (единицы 0.5%, т.е. 200 = 100%)
 */
uint8_t battery_monitor_get_zb_percentage_remaining(void);

#ifdef __cplusplus
}
#endif
