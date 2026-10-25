/*
 * SPDX-FileCopyrightText: 2026 Antigravity
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ESP-NOW -> Zigbee 3.0 Bridge Receiver Implementation
 */

#include "sdkconfig.h"

#if CONFIG_ESPNOW_BRIDGE_ENABLE

#include "espnow_bridge.h"
#include <string.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#ifndef CONFIG_ESPNOW_LOG_PACKETS
#define CONFIG_ESPNOW_LOG_PACKETS 0
#endif

#if !CONFIG_ESPNOW_LOG_PACKETS
#undef ESP_LOGI
#define ESP_LOGI(tag, fmt, ...) ((void)0)
#endif

static const char *TAG = "ESPNOW_BRIDGE";

static espnow_bridge_cmd_cb_t s_cmd_cb = NULL;
static QueueHandle_t s_espnow_queue = NULL;

static void espnow_worker_task(void *pvParameters)
{
    espnow_switch_pkt_t pkt;
    while (1) {
        if (xQueueReceive(s_espnow_queue, &pkt, pdMS_TO_TICKS(5000)) == pdTRUE) {
            ESP_LOGI(TAG, "📩 Принят пакет с выключателя: Линия %d, Действие %d (%s), Батарея C3: %d мВ",
                     pkt.line_id, pkt.action,
                     pkt.action == ESPNOW_ACTION_OFF ? "ВЫКЛ" :
                     pkt.action == ESPNOW_ACTION_ON ? "ВКЛ" : "TOGGLE",
                     pkt.vdd_mv);

            if (s_cmd_cb) {
                s_cmd_cb(pkt.line_id, pkt.action, pkt.vdd_mv);
            }
        } else {
            uint8_t ch = 0;
            wifi_second_chan_t sec;
            esp_wifi_get_channel(&ch, &sec);
            ESP_LOGI(TAG, "⏳ [ESPNOW Мост] Слушаем эфир... Текущий Wi-Fi канал: %d", ch);
        }
    }
}

static uint8_t s_last_pkt_seq = 0;
static uint32_t s_last_pkt_tick = 0;

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int data_len)
{
    if (!recv_info || !data || data_len < (int)sizeof(espnow_switch_pkt_t)) {
        return;
    }

    const espnow_switch_pkt_t *pkt = (const espnow_switch_pkt_t *)data;
    if (pkt->magic != ESPNOW_SWITCH_MAGIC) {
        return;
    }

    int8_t rssi = recv_info->rx_ctrl ? recv_info->rx_ctrl->rssi : 0;
    ESP_LOGW("RADIO_RSSI", "📶 [RSSI C3->C6]: %d dBm | Линия: %d | Действие: %d | Seq: %d",
             rssi, pkt->line_id, pkt->action, pkt->seq);

    // Дедупликация пакетов из одной радио-пачки (окно 350 мс)
    uint32_t now = (uint32_t)xTaskGetTickCount();
    if (pkt->seq == s_last_pkt_seq && (now - s_last_pkt_tick) < pdMS_TO_TICKS(350)) {
        return; // Повторный пакет серии для пробития радио-окна
    }
    s_last_pkt_seq = pkt->seq;
    s_last_pkt_tick = now;

    if (s_espnow_queue) {
        xQueueSend(s_espnow_queue, pkt, 0);
    }
}

esp_err_t espnow_bridge_init(uint8_t channel, espnow_bridge_cmd_cb_t cmd_cb)
{
    s_cmd_cb = cmd_cb;

    s_espnow_queue = xQueueCreate(10, sizeof(espnow_switch_pkt_t));
    if (!s_espnow_queue) {
        ESP_LOGE(TAG, "Не удалось создать очередь команд ESP-NOW");
        return ESP_ERR_NO_MEM;
    }

    xTaskCreate(espnow_worker_task, "espnow_bridge_task", 4096, NULL, 6, NULL);

    ESP_ERROR_CHECK(esp_netif_init());
    esp_err_t err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    wifi_config_t ap_cfg = {
        .ap = {
            .ssid = "C6_HUB",
            .ssid_len = strlen("C6_HUB"),
            .channel = channel,
            .max_connection = 4,
            .authmode = WIFI_AUTH_OPEN,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    esp_wifi_set_max_tx_power(84); // Максимальная мощность 21 dBm

    ESP_ERROR_CHECK(esp_now_init());

    // Регистрируем широковещательного пира на AP интерфейсе
    esp_now_peer_info_t peer = {
        .channel = channel,
        .ifidx = WIFI_IF_AP,
        .encrypt = false,
    };
    memset(peer.peer_addr, 0xFF, 6);
    esp_now_add_peer(&peer);

    // Добавляем прямой пир для выключателя C3 (10:B4:1D:15:BF:54)
    uint8_t c3_mac[6] = {0x10, 0xB4, 0x1D, 0x15, 0xBF, 0x54};
    memcpy(peer.peer_addr, c3_mac, 6);
    esp_now_add_peer(&peer);

    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    ESP_LOGI(TAG, "🚀 Мост ESP-NOW запущен (канал %d, PS_NONE, пиры: Broadcast + C3). Ожидание сигналов...", channel);
    return ESP_OK;
}

#else // !CONFIG_ESPNOW_BRIDGE_ENABLE

#include "espnow_bridge.h"

esp_err_t espnow_bridge_init(uint8_t channel, espnow_bridge_cmd_cb_t cmd_cb)
{
    (void)channel;
    (void)cmd_cb;
    return ESP_OK;
}

#endif // CONFIG_ESPNOW_BRIDGE_ENABLE

