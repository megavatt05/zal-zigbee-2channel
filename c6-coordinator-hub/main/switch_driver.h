/*
 * SPDX-FileCopyrightText: 2026 Antigravity
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Драйвер выключателей для ESP32-C6:
 * - Внешняя Клавиша 1 (GPIO 6 без фиксации -> Управление Линией 1)
 * - Внешняя Клавиша 2 (GPIO 7 без фиксации -> Управление Линией 2)
 * - Сервисная кнопка BOOT (GPIO 9 -> Одиночный=Линия 1, Двойной=Линия 2, Удержание=Сопряжение)
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SWITCH_BOOT_GPIO   GPIO_NUM_9
#define SWITCH_KEY1_GPIO   GPIO_NUM_6
#define SWITCH_KEY2_GPIO   GPIO_NUM_7

/**
 * @brief Типы событий переключателей
 */
typedef enum {
    SWITCH_EVENT_LINE1_TOGGLE = 0, /*!< Переключение Линии 1 */
    SWITCH_EVENT_LINE2_TOGGLE,     /*!< Переключение Линии 2 */
    SWITCH_EVENT_PAIRING_OPEN,     /*!< Открыть сопряжение сети Zigbee */
} switch_event_t;

/**
 * @brief Callback функция обработки события переключателей
 */
typedef void (*switch_event_cb_t)(switch_event_t event, void *user_data);

/**
 * @brief Конфигурация выключателей
 */
typedef struct {
    gpio_num_t        gpio_boot;    /*!< GPIO кнопки BOOT (GPIO 9) */
    gpio_num_t        gpio_key1;    /*!< GPIO Клавиши 1 (GPIO 6) */
    gpio_num_t        gpio_key2;    /*!< GPIO Клавиши 2 (GPIO 7) */
    switch_event_cb_t event_cb;     /*!< Callback для событий */
    void             *user_data;    /*!< Пользовательский контекст */
} switch_driver_config_t;

/**
 * @brief Инициализация драйвера выключателей
 */
esp_err_t switch_driver_init(const switch_driver_config_t *config);

#ifdef __cplusplus
}
#endif
