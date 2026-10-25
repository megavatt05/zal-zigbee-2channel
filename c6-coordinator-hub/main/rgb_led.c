/*
 * SPDX-FileCopyrightText: 2026 Antigravity
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Надежный драйвер индикации WS2812 на базе FreeRTOS задачи
 */

#include "sdkconfig.h"

#if CONFIG_RGB_LED_ENABLE

#include "rgb_led.h"
#include "led_strip.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const char *TAG = "RGB_LED";

typedef enum {
    RGB_CMD_CLEAR,
    RGB_CMD_PAIRING_START,
    RGB_CMD_PAIRING_STOP,
    RGB_CMD_FLASH,
} rgb_cmd_type_t;

typedef struct {
    rgb_cmd_type_t type;
    uint8_t        r;
    uint8_t        g;
    uint8_t        b;
    uint32_t       duration_ms;
} rgb_cmd_t;

static led_strip_handle_t s_led_strip = NULL;
static QueueHandle_t      s_cmd_queue = NULL;

static void rgb_led_task(void *pvParameters)
{
    rgb_cmd_t cmd;
    bool pairing_active = false;
    bool led_state = false;

    ESP_LOGI(TAG, "Задача управления RGB-светодиодом запущена");

    while (1) {
        // Если идёт поиск сети (pairing), просыпаемся каждые 400 мс для мигания.
        // Если режим покоя — блокируемся насовсем (portMAX_DELAY), позволяя процессору спать в Light Sleep!
        TickType_t wait_ticks = pairing_active ? pdMS_TO_TICKS(400) : portMAX_DELAY;

        if (xQueueReceive(s_cmd_queue, &cmd, wait_ticks) == pdTRUE) {
            switch (cmd.type) {
            case RGB_CMD_FLASH:
                // Яркая вспышка заданным цветом
                led_strip_set_pixel(s_led_strip, 0, cmd.r, cmd.g, cmd.b);
                led_strip_refresh(s_led_strip);
                vTaskDelay(pdMS_TO_TICKS(cmd.duration_ms));
                if (!pairing_active) {
                    led_strip_clear(s_led_strip);
                } else {
                    // Возврат к янтарному
                    led_strip_set_pixel(s_led_strip, 0, 200, 60, 0);
                    led_strip_refresh(s_led_strip);
                    led_state = true;
                }
                break;

            case RGB_CMD_PAIRING_START:
                pairing_active = true;
                led_state = true;
                ESP_LOGI(TAG, ">>> Индикация: ВКЛЮЧЕНО янтарное мигание поиска сети <<<");
                led_strip_set_pixel(s_led_strip, 0, 200, 60, 0); // Яркий янтарный
                led_strip_refresh(s_led_strip);
                break;

            case RGB_CMD_PAIRING_STOP:
                pairing_active = false;
                ESP_LOGI(TAG, ">>> Индикация: ВЫКЛЮЧЕНО янтарное мигание <<<");
                led_strip_clear(s_led_strip);
                break;

            case RGB_CMD_CLEAR:
                pairing_active = false;
                led_strip_clear(s_led_strip);
                break;
            }
        } else {
            // Сработал таймаут 400 мс во время сопряжения — переключаем состояние
            if (pairing_active) {
                led_state = !led_state;
                if (led_state) {
                    led_strip_set_pixel(s_led_strip, 0, 200, 60, 0); // Яркий янтарный
                    led_strip_refresh(s_led_strip);
                } else {
                    led_strip_clear(s_led_strip);
                }
            }
        }
    }
}

esp_err_t rgb_led_init(gpio_num_t gpio_num)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = gpio_num,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags = {
            .invert_out = false,
        }
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .flags = {
            .with_dma = false,
        }
    };

    esp_err_t ret = led_strip_new_rmt_device(&strip_config, &rmt_config, &s_led_strip);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка инициализации RMT LED strip на GPIO %d: %s", gpio_num, esp_err_to_name(ret));
        return ret;
    }

    led_strip_clear(s_led_strip);

    s_cmd_queue = xQueueCreate(8, sizeof(rgb_cmd_t));
    xTaskCreate(rgb_led_task, "rgb_led_task", 3072, NULL, 6, NULL);

    ESP_LOGI(TAG, "RGB WS2812 инициализирован на GPIO %d (управление через FreeRTOS очередь)", gpio_num);
    return ESP_OK;
}

esp_err_t rgb_led_set_color(uint8_t red, uint8_t green, uint8_t blue)
{
    if (!s_cmd_queue) return ESP_ERR_INVALID_STATE;
    rgb_cmd_t cmd = {
        .type = RGB_CMD_FLASH,
        .r = red,
        .g = green,
        .b = blue,
        .duration_ms = 0,
    };
    return xQueueSend(s_cmd_queue, &cmd, 0) == pdTRUE ? ESP_OK : ESP_FAIL;
}

esp_err_t rgb_led_clear(void)
{
    if (!s_cmd_queue) return ESP_ERR_INVALID_STATE;
    rgb_cmd_t cmd = { .type = RGB_CMD_CLEAR };
    return xQueueSend(s_cmd_queue, &cmd, 0) == pdTRUE ? ESP_OK : ESP_FAIL;
}

void rgb_led_flash(uint8_t red, uint8_t green, uint8_t blue, uint32_t duration_ms)
{
    if (!s_cmd_queue) return;
    rgb_cmd_t cmd = {
        .type = RGB_CMD_FLASH,
        .r = red,
        .g = green,
        .b = blue,
        .duration_ms = duration_ms,
    };
    xQueueSend(s_cmd_queue, &cmd, 0);
}

void rgb_led_set_pairing_mode(bool enable)
{
    if (!s_cmd_queue) return;
    rgb_cmd_t cmd = {
        .type = enable ? RGB_CMD_PAIRING_START : RGB_CMD_PAIRING_STOP,
    };
    xQueueSend(s_cmd_queue, &cmd, 0);
}

#else // !CONFIG_RGB_LED_ENABLE

#include "rgb_led.h"

esp_err_t rgb_led_init(gpio_num_t gpio_num) { (void)gpio_num; return ESP_OK; }
esp_err_t rgb_led_set_color(uint8_t red, uint8_t green, uint8_t blue) { (void)red; (void)green; (void)blue; return ESP_OK; }
esp_err_t rgb_led_clear(void) { return ESP_OK; }
void rgb_led_flash(uint8_t red, uint8_t green, uint8_t blue, uint32_t duration_ms) { (void)red; (void)green; (void)blue; (void)duration_ms; }
void rgb_led_set_pairing_mode(bool enable) { (void)enable; }

#endif // CONFIG_RGB_LED_ENABLE

