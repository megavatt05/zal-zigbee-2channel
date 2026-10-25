/*
 * SPDX-FileCopyrightText: 2026 Antigravity
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ESP32-C3 Сверхнизкопотребляющий выключатель (Deep Sleep ~5 мкА + Мгновенный Burst 80 мс)
 *
 * Назначение клавиш:
 * • Клавиша 1: GPIO 2 -> Линия 1 (EP 1 на хабе C6)
 * • Клавиша 2: GPIO 3 -> Линия 2 (EP 2 на хабе C6)
 * • Кнопка BOOT: отключена
 */

#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Безопасные значения по умолчанию для Kconfig
#ifndef CONFIG_C3_LINE1_ENABLE
#define CONFIG_C3_LINE1_ENABLE 1
#endif

#ifndef CONFIG_C3_LINE2_ENABLE
#define CONFIG_C3_LINE2_ENABLE 1
#endif

#ifndef CONFIG_C3_LOG_ENABLE
#define CONFIG_C3_LOG_ENABLE 0
#endif

#if !CONFIG_C3_LOG_ENABLE
#undef ESP_LOGI
#define ESP_LOGI(tag, fmt, ...) ((void)0)
#undef ESP_LOGW
#define ESP_LOGW(tag, fmt, ...) ((void)0)
#endif

static const char *TAG = "C3_SWITCH";

#define SWITCH_KEY1_GPIO  GPIO_NUM_2
#define SWITCH_KEY2_GPIO  GPIO_NUM_3

#define ESPNOW_CHANNEL    1
#define ESPNOW_SWITCH_MAGIC 0xC3

typedef enum {
    ESPNOW_ACTION_OFF    = 0,
    ESPNOW_ACTION_ON     = 1,
    ESPNOW_ACTION_TOGGLE = 2,
} espnow_action_t;

typedef struct __attribute__((packed)) {
    uint8_t magic;      // 0xC3
    uint8_t line_id;    // 1 = Линия 1, 2 = Линия 2
    uint8_t action;     // 0 = OFF, 1 = ON, 2 = TOGGLE
    uint8_t seq;        // Счетчик нажатий
    uint16_t vdd_mv;    // Напряжение питания (мВ)
} espnow_switch_pkt_t;

static uint8_t s_broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static uint8_t s_c6_unicast_mac[6] = {0x40, 0x4C, 0xCA, 0x55, 0x6C, 0x75}; // SoftAP MAC ESP32-C6

// Переменные в RTC памяти, сохраняющиеся между циклами глубокого сна
static RTC_DATA_ATTR uint8_t s_pkt_seq = 0;
static RTC_DATA_ATTR uint32_t s_boot_count = 0;

static void send_espnow_switch_command(uint8_t line_id, espnow_action_t action)
{
    espnow_switch_pkt_t pkt = {
        .magic = ESPNOW_SWITCH_MAGIC,
        .line_id = line_id,
        .action = action,
        .seq = ++s_pkt_seq,
        .vdd_mv = 3300,
    };

    ESP_LOGI(TAG, "📡 Отправка серии команды: Линия %d -> TOGGLE (Seq: %d)", line_id, pkt.seq);

    // Серия из 8 импульсов с интервалом 10 мс (суммарно ровно 80 мс Burst)
    // Гарантированно пробивает временные TDM-окна Coexistence радио ESP32-C6 (Zigbee/Wi-Fi)
    for (int i = 0; i < 8; i++) {
        esp_now_send(s_broadcast_mac, (uint8_t *)&pkt, sizeof(pkt));
        esp_now_send(s_c6_unicast_mac, (uint8_t *)&pkt, sizeof(pkt));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
    s_boot_count++;

    // 1. Инициализация и считывание пинов клавиш с подтяжкой
    uint64_t pin_mask = 0;
#if CONFIG_C3_LINE1_ENABLE
    pin_mask |= (1ULL << SWITCH_KEY1_GPIO);
#endif
#if CONFIG_C3_LINE2_ENABLE
    pin_mask |= (1ULL << SWITCH_KEY2_GPIO);
#endif

    if (pin_mask > 0) {
        gpio_config_t io_conf = {
            .pin_bit_mask = pin_mask,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io_conf);
    }

    // 2. Определение причины пробуждения и нажатой клавиши
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    uint8_t target_line = 0;

    if (cause == ESP_SLEEP_WAKEUP_GPIO) {
        uint64_t wakeup_mask = esp_sleep_get_gpio_wakeup_status();
#if CONFIG_C3_LINE1_ENABLE
        if (wakeup_mask & (1ULL << SWITCH_KEY1_GPIO)) {
            target_line = 1;
        }
#endif
#if CONFIG_C3_LINE2_ENABLE
        if (wakeup_mask & (1ULL << SWITCH_KEY2_GPIO)) {
            target_line = 2;
        }
#endif
    }

    // Резервная проверка уровней пинов (на случай если маска пуста или при удержании)
    if (target_line == 0) {
#if CONFIG_C3_LINE1_ENABLE
        if (gpio_get_level(SWITCH_KEY1_GPIO) == 0) {
            target_line = 1;
        }
#endif
#if CONFIG_C3_LINE2_ENABLE
        if (gpio_get_level(SWITCH_KEY2_GPIO) == 0) {
            target_line = 2;
        }
#endif
    }

    // 3. Если клавиша нажата — активируем радиосеть и отправляем пачку Burst
    if (target_line != 0) {
        ESP_LOGI(TAG, "🔘 ПРОБУЖДЕНИЕ: Нажата Клавиша %d! Отправка пакетов...", target_line);

        // Быстрая инициализация радиосети
        nvs_flash_init();
        esp_netif_init();
        esp_event_loop_create_default();

        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        esp_wifi_init(&cfg);
        esp_wifi_set_storage(WIFI_STORAGE_RAM);
        esp_wifi_set_mode(WIFI_MODE_STA);
        esp_wifi_start();
        esp_wifi_set_ps(WIFI_PS_NONE);
        esp_wifi_set_max_tx_power(84); // 21 dBm
        esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

        esp_now_init();

        esp_now_peer_info_t peer = {
            .channel = ESPNOW_CHANNEL,
            .ifidx = WIFI_IF_STA,
            .encrypt = false,
        };
        memcpy(peer.peer_addr, s_broadcast_mac, 6);
        esp_now_add_peer(&peer);

        memcpy(peer.peer_addr, s_c6_unicast_mac, 6);
        esp_now_add_peer(&peer);

        // Отправка серии Burst 80 мс
        send_espnow_switch_command(target_line, ESPNOW_ACTION_TOGGLE);

        // Отключение радио для экономии заряда
        esp_now_deinit();
        esp_wifi_stop();
        esp_wifi_deinit();

        // Ожидание отпускания кнопки (до 300 мс) с антидребезгом
        int wait_release = 30;
        while (wait_release-- > 0) {
            bool pressed = false;
#if CONFIG_C3_LINE1_ENABLE
            if (gpio_get_level(SWITCH_KEY1_GPIO) == 0) pressed = true;
#endif
#if CONFIG_C3_LINE2_ENABLE
            if (gpio_get_level(SWITCH_KEY2_GPIO) == 0) pressed = true;
#endif
            if (!pressed) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        vTaskDelay(pdMS_TO_TICKS(30)); // Антидребезг при размыкании
    }

    // 4. Подготовка к глубокому сну и фиксация подтяжки Pull-Up
    if (pin_mask > 0) {
#if CONFIG_C3_LINE1_ENABLE
        gpio_pullup_en(SWITCH_KEY1_GPIO);
        gpio_pulldown_dis(SWITCH_KEY1_GPIO);
        gpio_hold_en(SWITCH_KEY1_GPIO);
#endif
#if CONFIG_C3_LINE2_ENABLE
        gpio_pullup_en(SWITCH_KEY2_GPIO);
        gpio_pulldown_dis(SWITCH_KEY2_GPIO);
        gpio_hold_en(SWITCH_KEY2_GPIO);
#endif
        gpio_deep_sleep_hold_en();

        ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup_on_hp_periph_powerdown(pin_mask, ESP_GPIO_WAKEUP_GPIO_LOW));
    }

    ESP_LOGI(TAG, "💤 Переход в Deep Sleep (~5 мкА)... Ожидание нажатия.");
    esp_deep_sleep_start();
}
