/*
 * SPDX-FileCopyrightText: 2026 Antigravity
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Драйвер выключателей для ESP32-C6 (GPIO 6, GPIO 7 и BOOT GPIO 9)
 */

#include "sdkconfig.h"

#if CONFIG_SWITCH_DRIVER_ENABLE

#include "switch_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "SWITCH_DRIVER";

static switch_driver_config_t s_config;
static QueueHandle_t s_gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)(uintptr_t)arg;
    xQueueSendFromISR(s_gpio_evt_queue, &gpio_num, NULL);
}

static void button_task(void *pvParameters)
{
    uint32_t io_num;
    for (;;) {
        if (xQueueReceive(s_gpio_evt_queue, &io_num, portMAX_DELAY)) {
            vTaskDelay(pdMS_TO_TICKS(25)); // Первичный антидребезг

            // --- Обработка внешней клавиши 1 (GPIO 6) ---
            if (io_num == (uint32_t)s_config.gpio_key1) {
                if (gpio_get_level(s_config.gpio_key1) == 0) {
                    ESP_LOGI(TAG, "Внешняя Клавиша 1 (GPIO %d) нажата -> Линия 1 Toggle", s_config.gpio_key1);
                    if (s_config.event_cb) {
                        s_config.event_cb(SWITCH_EVENT_LINE1_TOGGLE, s_config.user_data);
                    }
                    // Ждем отпускания клавиши
                    while (gpio_get_level(s_config.gpio_key1) == 0) {
                        vTaskDelay(pdMS_TO_TICKS(30));
                    }
                    vTaskDelay(pdMS_TO_TICKS(50));
                }
                xQueueReset(s_gpio_evt_queue);
                continue;
            }

            // --- Обработка внешней клавиши 2 (GPIO 7) ---
            if (io_num == (uint32_t)s_config.gpio_key2) {
                if (gpio_get_level(s_config.gpio_key2) == 0) {
                    ESP_LOGI(TAG, "Внешняя Клавиша 2 (GPIO %d) нажата -> Линия 2 Toggle", s_config.gpio_key2);
                    if (s_config.event_cb) {
                        s_config.event_cb(SWITCH_EVENT_LINE2_TOGGLE, s_config.user_data);
                    }
                    // Ждем отпускания клавиши
                    while (gpio_get_level(s_config.gpio_key2) == 0) {
                        vTaskDelay(pdMS_TO_TICKS(30));
                    }
                    vTaskDelay(pdMS_TO_TICKS(50));
                }
                xQueueReset(s_gpio_evt_queue);
                continue;
            }

            // --- Обработка сервисной кнопки BOOT (GPIO 9) ---
            if (io_num == (uint32_t)s_config.gpio_boot) {
                if (gpio_get_level(s_config.gpio_boot) != 0) {
                    continue;
                }

                int64_t press_start = esp_timer_get_time();
                bool long_press_fired = false;

                // Ждем отпускания или удержания > 2.5 сек
                while (gpio_get_level(s_config.gpio_boot) == 0) {
                    vTaskDelay(pdMS_TO_TICKS(50));
                    int64_t held_time_ms = (esp_timer_get_time() - press_start) / 1000;

                    if (!long_press_fired && held_time_ms >= 2500) {
                        long_press_fired = true;
                        ESP_LOGI(TAG, "Кнопка BOOT: Длинное нажатие (> 2.5 сек) -> Открытие сети для сопряжения");
                        if (s_config.event_cb) {
                            s_config.event_cb(SWITCH_EVENT_PAIRING_OPEN, s_config.user_data);
                        }
                        break;
                    }
                }

                if (long_press_fired) {
                    while (gpio_get_level(s_config.gpio_boot) == 0) {
                        vTaskDelay(pdMS_TO_TICKS(50));
                    }
                    vTaskDelay(pdMS_TO_TICKS(100));
                    xQueueReset(s_gpio_evt_queue);
                    continue;
                }

                // Быстрое отпускание: проверяем возможность двойного клика (окно 300 мс)
                bool double_click = false;
                int64_t release_time = esp_timer_get_time();

                while ((esp_timer_get_time() - release_time) < 300000) {
                    if (gpio_get_level(s_config.gpio_boot) == 0) {
                        vTaskDelay(pdMS_TO_TICKS(25));
                        if (gpio_get_level(s_config.gpio_boot) == 0) {
                            double_click = true;
                            while (gpio_get_level(s_config.gpio_boot) == 0) {
                                vTaskDelay(pdMS_TO_TICKS(30));
                            }
                            vTaskDelay(pdMS_TO_TICKS(50));
                            break;
                        }
                    }
                    vTaskDelay(pdMS_TO_TICKS(20));
                }

                if (double_click) {
                    ESP_LOGI(TAG, "Кнопка BOOT: Двойной клик -> Линия 2 Toggle");
                    if (s_config.event_cb) {
                        s_config.event_cb(SWITCH_EVENT_LINE2_TOGGLE, s_config.user_data);
                    }
                } else {
                    ESP_LOGI(TAG, "Кнопка BOOT: Одиночный клик -> Линия 1 Toggle");
                    if (s_config.event_cb) {
                        s_config.event_cb(SWITCH_EVENT_LINE1_TOGGLE, s_config.user_data);
                    }
                }

                xQueueReset(s_gpio_evt_queue);
            }
        }
    }
}

esp_err_t switch_driver_init(const switch_driver_config_t *config)
{
    if (!config) return ESP_ERR_INVALID_ARG;
    s_config = *config;

    // Настройка пинов: Внутренняя подтяжка к 3.3V, срабатывание по спаду (клик на GND)
    uint64_t pin_mask = (1ULL << s_config.gpio_boot) |
                        (1ULL << s_config.gpio_key1) |
                        (1ULL << s_config.gpio_key2);

    gpio_config_t io_conf = {
        .pin_bit_mask = pin_mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    s_gpio_evt_queue = xQueueCreate(8, sizeof(uint32_t));
    xTaskCreate(button_task, "switch_task", 3072, NULL, 10, NULL);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(s_config.gpio_boot, gpio_isr_handler, (void *)(uintptr_t)s_config.gpio_boot);
    gpio_isr_handler_add(s_config.gpio_key1, gpio_isr_handler, (void *)(uintptr_t)s_config.gpio_key1);
    gpio_isr_handler_add(s_config.gpio_key2, gpio_isr_handler, (void *)(uintptr_t)s_config.gpio_key2);

    ESP_LOGI(TAG, "Драйвер выключателей настроен:");
    ESP_LOGI(TAG, "  • Клавиша 1: GPIO %d -> Линия 1", s_config.gpio_key1);
    ESP_LOGI(TAG, "  • Клавиша 2: GPIO %d -> Линия 2", s_config.gpio_key2);
    ESP_LOGI(TAG, "  • Кнопка BOOT: GPIO %d (1 клик = Линия 1, 2 клика = Линия 2, удержание = сопряжение)", s_config.gpio_boot);

    return ESP_OK;
}

#else // !CONFIG_SWITCH_DRIVER_ENABLE

#include "switch_driver.h"

esp_err_t switch_driver_init(const switch_driver_config_t *config)
{
    (void)config;
    return ESP_OK;
}

#endif // CONFIG_SWITCH_DRIVER_ENABLE

