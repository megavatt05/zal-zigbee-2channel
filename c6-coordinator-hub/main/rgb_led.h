/*
 * SPDX-FileCopyrightText: 2026 Antigravity
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Драйвер адресного RGB-светодиода (WS2812) для энергоэффективной индикации
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RGB_LED_DEFAULT_GPIO GPIO_NUM_8

/**
 * @brief Инициализация RGB светодиода (WS2812)
 */
esp_err_t rgb_led_init(gpio_num_t gpio_num);

/**
 * @brief Установка цвета RGB (яркость от 0 до 255)
 */
esp_err_t rgb_led_set_color(uint8_t red, uint8_t green, uint8_t blue);

/**
 * @brief Полное отключение светодиода (0 мкА по шине данных)
 */
esp_err_t rgb_led_clear(void);

/**
 * @brief Кратковременная вспышка заданным цветом (с автоотключением)
 */
void rgb_led_flash(uint8_t red, uint8_t green, uint8_t blue, uint32_t duration_ms);

/**
 * @brief Включение/выключение индикации поиска сети Zigbee (мигание)
 */
void rgb_led_set_pairing_mode(bool enable);

#ifdef __cplusplus
}
#endif
