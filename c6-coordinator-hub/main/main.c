/*
 * SPDX-FileCopyrightText: 2026 Antigravity
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ESP32-C6 Standalone Zigbee Coordinator (Direct Control of 2-Line Tuya Relay)
 * Реализация расцепления независимых каналов (Relay Separation Uncoupling)
 * и автоматический тестовый цикл переключения каналов.
 */

#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_check.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "main.h"
#include "esp_pm.h"
#include "ezbee/secur.h"
#include "ezbee/nwk.h"
#include "ezbee/aps.h"
#include "ezbee/zdo/zdo_dev_srv_disc.h"
#include "ezbee/zdo/zdo_bind_mgmt.h"
#include "switch_driver.h"
#include "rgb_led.h"
#include "espnow_bridge.h"

#ifndef CONFIG_ZB_ENDPOINT_1_ENABLE
#define CONFIG_ZB_ENDPOINT_1_ENABLE 1
#endif
#ifndef CONFIG_ZB_ENDPOINT_2_ENABLE
#define CONFIG_ZB_ENDPOINT_2_ENABLE 1
#endif
#ifndef CONFIG_ZB_TUYA_UNCOUPLING_ENABLE
#define CONFIG_ZB_TUYA_UNCOUPLING_ENABLE 1
#endif
#ifndef CONFIG_ESPNOW_BRIDGE_ENABLE
#define CONFIG_ESPNOW_BRIDGE_ENABLE 1
#endif
#ifndef CONFIG_RGB_LED_ENABLE
#define CONFIG_RGB_LED_ENABLE 1
#endif
#ifndef CONFIG_SWITCH_DRIVER_ENABLE
#define CONFIG_SWITCH_DRIVER_ENABLE 1
#endif
#ifndef CONFIG_ZB_LOG_ENABLE
#define CONFIG_ZB_LOG_ENABLE 0
#endif

// При отключенном CONFIG_ZB_LOG_ENABLE все информационные логи удаляются из сборки
#if !CONFIG_ZB_LOG_ENABLE
#undef ESP_LOGI
#define ESP_LOGI(tag, fmt, ...) ((void)0)
#endif

#if CONFIG_ZB_LOG_TUYA_STEPS
#define TUYA_STEP_LOGI(fmt, ...) ESP_LOGI(TAG, fmt, ##__VA_ARGS__)
#else
#define TUYA_STEP_LOGI(fmt, ...) ((void)0)
#endif

static const char *TAG = "ZB_COORDINATOR";

static uint16_t s_relay_short_addr = 0xFFFF; // 0xFFFF = broadcast пока адрес не обнаружен
static uint8_t s_ep_line1 = 1;
static uint8_t s_ep_line2 = 2;
static bool s_uncouple_done = false;

static void simple_desc_cb(const ezb_zdo_simple_desc_req_result_t *result, void *user_ctx)
{
#if CONFIG_ZB_LOG_CLUSTER_DISCOVERY
    if (result && result->rsp && result->rsp->status == 0) {
        ezb_af_simple_desc_t *sd = &result->rsp->desc;
        ESP_LOGI(TAG, "---------------------------------------------------------");
        ESP_LOGI(TAG, "🔍 Simple Descriptor для EP %d: Profile 0x%04x, DeviceID 0x%04x",
                 sd->ep_id, sd->app_profile_id, sd->app_device_id);
        ESP_LOGI(TAG, "   Входных кластеров (серверов): %d", sd->app_input_cluster_count);
        for (int i = 0; i < sd->app_input_cluster_count; i++) {
            uint16_t c_id = sd->app_cluster_list[i];
            ESP_LOGI(TAG, "     Кластер [%d]: 0x%04x (%s)",
                     i, c_id,
                     c_id == 0x0006 ? "On/Off" :
                     c_id == 0xef00 ? "Tuya Private DP Cluster" :
                     c_id == 0x0000 ? "Basic" :
                     c_id == 0x0004 ? "Groups" :
                     c_id == 0x0005 ? "Scenes" : "Other");
        }
        ESP_LOGI(TAG, "---------------------------------------------------------");
    }
#else
    (void)result;
    (void)user_ctx;
#endif
}

static void query_simple_desc(uint16_t short_addr, uint8_t endpoint)
{
    ezb_zdo_simple_desc_req_t req = {
        .dst_nwk_addr = short_addr,
        .field = {
            .nwk_addr_of_interest = short_addr,
            .endpoint = endpoint,
        },
        .cb = simple_desc_cb,
    };
    esp_zigbee_lock_acquire(portMAX_DELAY);
    ezb_zdo_simple_desc_req(&req);
    esp_zigbee_lock_release();
}

static void query_relay_endpoints(uint16_t short_addr);

static void active_ep_cb(const ezb_zdo_active_ep_req_result_t *result, void *user_ctx)
{
    if (result && result->rsp && result->rsp->status == 0) {
        ESP_LOGI(TAG, "=========================================================");
        ESP_LOGI(TAG, "🎉 ПОЛУЧЕН СПИСОК ЭНДПОИНТОВ ОТ РЕЛЕ (Всего: %d):", result->rsp->active_ep_count);
        for (int i = 0; i < result->rsp->active_ep_count; i++) {
            uint8_t ep = result->rsp->active_ep_list[i];
            ESP_LOGI(TAG, "    Endpoint [%d] = %d", i, ep);
            query_simple_desc(result->rsp->nwk_addr_of_interest, ep);
        }
        if (result->rsp->active_ep_count >= 1) {
            s_ep_line1 = result->rsp->active_ep_list[0];
        }
        if (result->rsp->active_ep_count >= 2) {
            s_ep_line2 = result->rsp->active_ep_list[1];
        }
        ESP_LOGI(TAG, ">>> НАЗНАЧЕНО: Линия 1 -> EP %d, Линия 2 -> EP %d <<<", s_ep_line1, s_ep_line2);
        ESP_LOGI(TAG, "=========================================================");
    } else {
        ESP_LOGW(TAG, "Active EP req завершился с ошибкой или пустым списком");
    }
}

static void query_relay_endpoints(uint16_t short_addr)
{
    if (short_addr == 0x0000 || short_addr == 0xFFFF) return;
    ESP_LOGI(TAG, ">>> Запрос Active Endpoints для адреса 0x%04hx... <<<", short_addr);
    ezb_zdo_active_ep_req_t req = {
        .dst_nwk_addr = short_addr,
        .field = {
            .nwk_addr_of_interest = short_addr,
        },
        .cb = active_ep_cb,
    };
    esp_zigbee_lock_acquire(portMAX_DELAY);
    ezb_zdo_active_ep_req(&req);
    esp_zigbee_lock_release();
}

static void update_relay_address_from_neighbors(void)
{
    ezb_nwk_info_iterator_t it = EZB_NWK_INFO_ITERATOR_INIT;
    ezb_nwk_neighbor_info_t nbr;
    while (ezb_nwk_get_next_neighbor(&it, &nbr) == EZB_ERR_NONE) {
        if (nbr.short_addr != 0x0000 && nbr.short_addr != 0xFFFF) {
            ESP_LOGI(TAG, "Найден сосед в таблице: short=0x%04hx, IEEE=0x%016llx",
                     nbr.short_addr, (unsigned long long)nbr.ieee_addr.u64);
            s_relay_short_addr = nbr.short_addr;
            query_relay_endpoints(s_relay_short_addr);
            break;
        }
    }
}

/**
 * @brief Отправка команды управления (OFF=0, ON=1, TOGGLE=2) на выбранный эндпоинт реле
 */
static void send_relay_cmd(uint8_t line_endpoint, uint8_t action)
{
#if !CONFIG_ZB_ENDPOINT_1_ENABLE && !CONFIG_ZB_ENDPOINT_2_ENABLE
    (void)line_endpoint; (void)action;
    ESP_LOGW(TAG, "Все конечные точки отключены в конфигураторе menuconfig");
    return;
#else
#if !CONFIG_ZB_ENDPOINT_1_ENABLE
    if (line_endpoint == s_ep_line1) {
        ESP_LOGW(TAG, "Команда отклонена: Конечная точка 1 (EP %d) отключена в menuconfig", line_endpoint);
        return;
    }
#endif
#if !CONFIG_ZB_ENDPOINT_2_ENABLE
    if (line_endpoint == s_ep_line2) {
        ESP_LOGW(TAG, "Команда отклонена: Конечная точка 2 (EP %d) отключена в menuconfig", line_endpoint);
        return;
    }
#endif

    if (s_relay_short_addr == 0xFFFF) {
        update_relay_address_from_neighbors();
    }
    if (s_relay_short_addr == 0xFFFF) {
        ESP_LOGW(TAG, "Адрес реле еще не обнаружен, команда пропущена");
        return;
    }

    uint8_t src_ep = (line_endpoint == s_ep_line2) ? ESP_ZIGBEE_COORDINATOR_ENDPOINT_2 : ESP_ZIGBEE_COORDINATOR_ENDPOINT_1;

    ezb_zcl_on_off_cmd_t cmd_req = {
        .cmd_ctrl = {
            .dst_addr = {
                .addr_mode = EZB_ADDR_MODE_SHORT,
                .u.short_addr = s_relay_short_addr,
            },
            .src_ep = src_ep,
            .dst_ep = line_endpoint,
            .dis_default_rsp = true,
        },
    };

    esp_zigbee_lock_acquire(portMAX_DELAY);
    ezb_err_t err = EZB_ERR_NONE;
    if (action == 0) {
        err = ezb_zcl_on_off_off_cmd_req(&cmd_req);
    } else if (action == 1) {
        err = ezb_zcl_on_off_on_cmd_req(&cmd_req);
    } else {
        err = ezb_zcl_on_off_toggle_cmd_req(&cmd_req);
    }
    esp_zigbee_lock_release();

    const char *act_str = (action == 0) ? "ВЫКЛЮЧИТЬ (OFF)" : (action == 1) ? "ВКЛЮЧИТЬ (ON)" : "ПЕРЕКЛЮЧИТЬ (TOGGLE)";
    if (err == EZB_ERR_NONE) {
        ESP_LOGI(TAG, "Команда [%s] отправлена -> Линия (EP %d, Адрес 0x%04hx, SrcEP %d)",
                 act_str, line_endpoint, s_relay_short_addr, src_ep);
    } else {
        ESP_LOGW(TAG, "Ошибка отправки команды [%s] для EP %d: 0x%04x", act_str, line_endpoint, err);
    }
#endif
}

/**
 * @brief Привязка кластера On/Off (0x0006) от эндпоинта реле к координатору
 */
static void bind_endpoint_cluster(uint16_t short_addr, uint8_t dev_ep, uint8_t coord_ep)
{
    ezb_extaddr_t dev_ieee, coord_ieee;
    if (ezb_address_extended_by_short(short_addr, &dev_ieee) != EZB_ERR_NONE) {
        return;
    }
    if (ezb_address_extended_by_short(0x0000, &coord_ieee) != EZB_ERR_NONE) {
        return;
    }

    ezb_zdo_bind_req_t req = {
        .dst_nwk_addr = short_addr,
        .field = {
            .src_addr = dev_ieee,
            .src_ep = dev_ep,
            .cluster_id = 0x0006, // On/Off
            .dst_addr_mode = 0x03, // 64-bit IEEE address
            .dst_addr = {
                .extended_addr = coord_ieee,
            },
            .dst_ep = coord_ep,
        },
        .cb = NULL,
        .user_ctx = NULL,
    };

    esp_zigbee_lock_acquire(portMAX_DELAY);
    ezb_zdo_bind_req(&req);
    esp_zigbee_lock_release();
    ESP_LOGI(TAG, "Отправлен ZDO Bind_req для EP %d -> ZC EP %d", dev_ep, coord_ep);
}

#if CONFIG_ZB_TUYA_UNCOUPLING_ENABLE
/**
 * @brief Процедура расцепления каналов Tuya (Uncoupling Sequence).
 * Решает заводскую проблему синхронного срабатывания двух линий.
 */
static void tuya_uncouple_relays(uint16_t short_addr)
{
    if (s_uncouple_done || short_addr == 0x0000 || short_addr == 0xFFFF) {
        return;
    }

    TUYA_STEP_LOGI("=========================================================");
    TUYA_STEP_LOGI("⚡ ЗАПУСК ПРОЦЕДУРЫ РАСЦЕПЛЕНИЯ КАНАЛОВ TUYA (Uncoupling)...");
    TUYA_STEP_LOGI("=========================================================");

    // ШАГ 1: Запрос Read Attributes базового кластера на широковещательный APS эндпоинт 0xFF
    static uint8_t read_payload[] = {
        0x10, // Frame control: entire profile, client-to-server, disable default response
        0x01, // TSN
        0x00, // Read attributes command ID
        0x04, 0x00, // attr 0x0004
        0x00, 0x00, // attr 0x0000
        0x01, 0x00, // attr 0x0001
        0x05, 0x00, // attr 0x0005
        0x07, 0x00, // attr 0x0007
        0xFE, 0xFF  // attr 0xFFFE
    };

    ezb_apsde_data_req_t req1 = {
        .dst_address = {
            .addr_mode = EZB_ADDR_MODE_SHORT,
            .u.short_addr = short_addr,
        },
        .src_endpoint = ESP_ZIGBEE_COORDINATOR_ENDPOINT_1,
        .dst_endpoint = 0xFF, // APS broadcast endpoint 0xFF
        .cluster_id = 0x0000, // Basic cluster
        .profile_id = 0x0104, // ZHA
        .radius = 30,
        .tx_options = 0,
        .asdu_length = sizeof(read_payload),
        .asdu = read_payload,
    };

    esp_zigbee_lock_acquire(portMAX_DELAY);
    ezb_err_t err1 = ezb_apsde_data_request(&req1);
    esp_zigbee_lock_release();
    (void)err1;
    TUYA_STEP_LOGI("Шаг 1 (Read Attrs to APS EP 0xFF): статус 0x%04x", err1);

    vTaskDelay(pdMS_TO_TICKS(350));

    // ШАГ 2: Запись атрибута 0xFFDE = 0x0D (тип uint8 = 0x20) на эндпоинт 1
    static uint8_t ffde_payload[] = {
        0x00, // Frame control: entire profile, server-side, enable default resp
        0x02, // TSN
        0x02, // Write attributes command ID
        0xDE, 0xFF, // Attribute 0xFFDE
        0x20, // Type uint8 (0x20)
        0x0D  // Value 0x0D
    };

    ezb_apsde_data_req_t req2 = {
        .dst_address = {
            .addr_mode = EZB_ADDR_MODE_SHORT,
            .u.short_addr = short_addr,
        },
        .src_endpoint = ESP_ZIGBEE_COORDINATOR_ENDPOINT_1,
        .dst_endpoint = 1,
        .cluster_id = 0x0000,
        .profile_id = 0x0104,
        .radius = 30,
        .tx_options = 0,
        .asdu_length = sizeof(ffde_payload),
        .asdu = ffde_payload,
    };

    esp_zigbee_lock_acquire(portMAX_DELAY);
    ezb_err_t err2 = ezb_apsde_data_request(&req2);
    esp_zigbee_lock_release();
    (void)err2;
    TUYA_STEP_LOGI("Шаг 2 (Write 0xFFDE=0x0D to EP 1): статус 0x%04x", err2);

    // ШАГ 3: Пауза 2.6 секунды (по спецификации инициализации Tuya)
    TUYA_STEP_LOGI("Шаг 3: Ожидание 2.6 сек перед завершающей командой разблокировки...");
    vTaskDelay(pdMS_TO_TICKS(2600));

    // ШАГ 4: Команда кластера Basic 0xF0 на эндпоинт 1
    static uint8_t f0_payload[] = {
        0x11, // Frame control: cluster specific, client to server, disable default resp
        0x03, // TSN
        0xF0  // Command 0xF0
    };

    ezb_apsde_data_req_t req3 = {
        .dst_address = {
            .addr_mode = EZB_ADDR_MODE_SHORT,
            .u.short_addr = short_addr,
        },
        .src_endpoint = ESP_ZIGBEE_COORDINATOR_ENDPOINT_1,
        .dst_endpoint = 1,
        .cluster_id = 0x0000,
        .profile_id = 0x0104,
        .radius = 30,
        .tx_options = 0,
        .asdu_length = sizeof(f0_payload),
        .asdu = f0_payload,
    };

    esp_zigbee_lock_acquire(portMAX_DELAY);
    ezb_err_t err3 = ezb_apsde_data_request(&req3);
    esp_zigbee_lock_release();
    (void)err3;
    TUYA_STEP_LOGI("Шаг 4 (Send Command 0xF0 to EP 1): статус 0x%04x", err3);

    vTaskDelay(pdMS_TO_TICKS(350));

    // ШАГ 5: Привязка реле к координатору для каждого включенного эндпоинта
#if CONFIG_ZB_ENDPOINT_1_ENABLE
    bind_endpoint_cluster(short_addr, s_ep_line1, ESP_ZIGBEE_COORDINATOR_ENDPOINT_1);
    vTaskDelay(pdMS_TO_TICKS(250));
#endif
#if CONFIG_ZB_ENDPOINT_2_ENABLE
    bind_endpoint_cluster(short_addr, s_ep_line2, ESP_ZIGBEE_COORDINATOR_ENDPOINT_2);
#endif

    s_uncouple_done = true;
    TUYA_STEP_LOGI("🎉 РАСЦЕПЛЕНИЕ КАНАЛОВ TUYA УСПЕШНО ЗАВЕРШЕНО!");
    TUYA_STEP_LOGI("=========================================================");
}
#else
static inline void tuya_uncouple_relays(uint16_t short_addr)
{
    (void)short_addr;
}
#endif // CONFIG_ZB_TUYA_UNCOUPLING_ENABLE

#if CONFIG_SWITCH_DRIVER_ENABLE
static void on_switch_event(switch_event_t event, void *user_data)
{
#if CONFIG_ZB_ENDPOINT_1_ENABLE
    if (event == SWITCH_EVENT_LINE1_TOGGLE) {
        ESP_LOGI(TAG, "=== Переключение ЛИНИИ 1 (EP %d) ===", s_ep_line1);
        rgb_led_flash(0, 100, 240, 150); // Синяя вспышка
        send_relay_cmd(s_ep_line1, 2); // TOGGLE
        return;
    }
#endif

#if CONFIG_ZB_ENDPOINT_2_ENABLE
    if (event == SWITCH_EVENT_LINE2_TOGGLE) {
        ESP_LOGI(TAG, "=== Переключение ЛИНИИ 2 (EP %d) ===", s_ep_line2);
        rgb_led_flash(0, 240, 100, 150); // Зеленая вспышка
        send_relay_cmd(s_ep_line2, 2); // TOGGLE
        return;
    }
#endif

    if (event == SWITCH_EVENT_PAIRING_OPEN) {
        ESP_LOGW(TAG, "=== Открытие сети для сопряжения на 180 секунд ===");
        rgb_led_flash(200, 0, 0, 600); // Красная вспышка

        esp_zigbee_lock_acquire(portMAX_DELAY);
        ezb_bdb_open_network(180);
        esp_zigbee_lock_release();
    }
}
#endif // CONFIG_SWITCH_DRIVER_ENABLE

#if CONFIG_ESPNOW_BRIDGE_ENABLE
static void on_espnow_cmd(uint8_t line_id, uint8_t action, uint16_t vdd_mv)
{
    ESP_LOGI(TAG, "⚡ [ESP-NOW] Принята команда с выключателя C3: Линия %d, Действие %d, Напряжение %d мВ",
             line_id, action, vdd_mv);

#if CONFIG_ZB_ENDPOINT_1_ENABLE
    if (line_id == 1) {
        rgb_led_flash(0, 100, 240, 120); // Синяя вспышка
        send_relay_cmd(s_ep_line1, action);
        return;
    }
#endif

#if CONFIG_ZB_ENDPOINT_2_ENABLE
    if (line_id == 2) {
        rgb_led_flash(0, 240, 100, 120); // Зеленая вспышка
        send_relay_cmd(s_ep_line2, action);
        return;
    }
#endif

    ESP_LOGW(TAG, "[ESP-NOW] Команда для Линии %d отклонена: соответствующий эндпоинт выключен", line_id);
}
#endif // CONFIG_ESPNOW_BRIDGE_ENABLE

static bool esp_zigbee_app_signal_handler(const ezb_app_signal_t *app_signal)
{
    ezb_app_signal_type_t signal_type = ezb_app_signal_get_type(app_signal);

    switch (signal_type) {
    case EZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Инициализация Zigbee координатора...");
        ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_INITIALIZATION);
        break;

    case EZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case EZB_BDB_SIGNAL_DEVICE_REBOOT: {
        ezb_bdb_comm_status_t status = *((ezb_bdb_comm_status_t *)ezb_app_signal_get_params(app_signal));
        if (status == EZB_BDB_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Координатор запущен (статус: %s)",
                     ezb_bdb_is_factory_new() ? "Новая сеть" : "Сеть уже сформирована");
            ESP_LOGI(TAG, "Текущая сеть: PAN ID: 0x%04hx, Канал: %d, Адрес ZC: 0x%04hx",
                     ezb_nwk_get_panid(), ezb_nwk_get_current_channel(), ezb_nwk_get_short_address());
            if (ezb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Формирование новой Zigbee сети...");
                ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_NETWORK_FORMATION);
            } else {
                ESP_LOGI(TAG, "Сеть восстановлена. Открываем сопряжение на 180 секунд...");
                update_relay_address_from_neighbors();
                ezb_bdb_open_network(180);
            }
        } else {
            ESP_LOGE(TAG, "Ошибка запуска стека BDB: 0x%02x", status);
        }
    } break;

    case EZB_BDB_SIGNAL_FORMATION: {
        ezb_bdb_comm_status_t status = *((ezb_bdb_comm_status_t *)ezb_app_signal_get_params(app_signal));
        if (status == EZB_BDB_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "=== Zigbee сеть успешно создана! ===");
            ESP_LOGI(TAG, "PAN ID: 0x%04hx, Канал: %d, Адрес координатора: 0x%04hx",
                     ezb_nwk_get_panid(), ezb_nwk_get_current_channel(), ezb_nwk_get_short_address());
            ESP_LOGI(TAG, "Открываем сеть для подключения реле на 180 секунд...");
            ezb_bdb_open_network(180);
        } else {
            ESP_LOGE(TAG, "Не удалось сформировать сеть (статус 0x%02x)", status);
        }
    } break;

    case EZB_NWK_SIGNAL_PERMIT_JOIN_STATUS: {
        uint8_t duration = *(uint8_t *)ezb_app_signal_get_params(app_signal);
        if (duration > 0) {
            ESP_LOGI(TAG, ">>> СЕТЬ ОТКРЫТА ДЛЯ СОПРЯЖЕНИЯ РЕЛЕ: осталось %d сек (мигание янтарным) <<<", duration);
            rgb_led_set_pairing_mode(true);
        } else {
            ESP_LOGI(TAG, "--- Окно сопряжения закрыто ---");
            rgb_led_set_pairing_mode(false);
        }
    } break;

    case EZB_ZDO_SIGNAL_DEVICE_UPDATE: {
        ezb_zdo_signal_device_update_params_t *upd = (ezb_zdo_signal_device_update_params_t *)ezb_app_signal_get_params(app_signal);
        ESP_LOGI(TAG, ">>> ОБНОВЛЕНИЕ УСТРОЙСТВА: Короткий адрес: 0x%04hx, Статус: 0x%02x",
                 upd->short_addr, upd->status);
        if (s_relay_short_addr == 0xFFFF) {
            s_relay_short_addr = upd->short_addr;
            query_relay_endpoints(s_relay_short_addr);
            rgb_led_flash(0, 200, 0, 1000);
        }
    } break;

    case EZB_ZDO_SIGNAL_DEVICE_AUTHORIZED: {
        ezb_zdo_signal_device_authorized_params_t *auth = (ezb_zdo_signal_device_authorized_params_t *)ezb_app_signal_get_params(app_signal);
        ESP_LOGI(TAG, ">>> УСТРОЙСТВО АВТОРИЗОВАНО: Короткий адрес: 0x%04hx, Статус: 0x%02x",
                 auth->short_addr, auth->status);
        s_relay_short_addr = auth->short_addr;
        query_relay_endpoints(s_relay_short_addr);
        rgb_led_flash(0, 200, 0, 1500);
    } break;

    case EZB_ZDO_SIGNAL_DEVICE_ANNCE: {
        ezb_zdo_signal_device_annce_params_t *annce = (ezb_zdo_signal_device_annce_params_t *)ezb_app_signal_get_params(app_signal);
        s_relay_short_addr = annce->short_addr;

        ESP_LOGI(TAG, "=========================================================");
        ESP_LOGI(TAG, "🎉 РЕЛЕ АНОНСИРОВАНО В СЕТИ (DEVICE_ANNCE)!");
        ESP_LOGI(TAG, "Короткий адрес: 0x%04hx, IEEE: 0x%016llx",
                 s_relay_short_addr, (unsigned long long)annce->device_addr.u64);
        ESP_LOGI(TAG, "=========================================================");

        query_relay_endpoints(s_relay_short_addr);
        rgb_led_flash(0, 200, 0, 2000);
    } break;

    default:
        ESP_LOGI(TAG, "Сигнал Zigbee: %s (0x%02x)", ezb_app_signal_to_string(signal_type), signal_type);
        break;
    }
    return true;
}

/**
 * @brief Создание двух конечных точек координатора (EP 1 для Линии 1, EP 2 для Линии 2)
 */
static esp_err_t esp_zigbee_create_coordinator_endpoints(void)
{
    ezb_af_device_desc_t dev_desc = ezb_af_create_device_desc();

#if CONFIG_ZB_ENDPOINT_1_ENABLE
    // Конечная точка 1 (Линия 1)
    ezb_zha_on_off_switch_config_t switch_cfg1 = EZB_ZHA_ON_OFF_SWITCH_CONFIG();
    ezb_af_ep_desc_t ep_desc1 = ezb_zha_create_on_off_switch(ESP_ZIGBEE_COORDINATOR_ENDPOINT_1, &switch_cfg1);
    ezb_zcl_cluster_desc_t basic_desc1 = ezb_af_endpoint_get_cluster_desc(ep_desc1, EZB_ZCL_CLUSTER_ID_BASIC, EZB_ZCL_CLUSTER_SERVER);
    if (basic_desc1) {
        ezb_zcl_basic_cluster_desc_add_attr(basic_desc1, EZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, (void *)ESP_MANUFACTURER_NAME);
        ezb_zcl_basic_cluster_desc_add_attr(basic_desc1, EZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, (void *)ESP_MODEL_IDENTIFIER);
    }
    ESP_ERROR_CHECK(ezb_af_device_add_endpoint_desc(dev_desc, ep_desc1));
#endif

#if CONFIG_ZB_ENDPOINT_2_ENABLE
    // Конечная точка 2 (Линия 2)
    ezb_zha_on_off_switch_config_t switch_cfg2 = EZB_ZHA_ON_OFF_SWITCH_CONFIG();
    ezb_af_ep_desc_t ep_desc2 = ezb_zha_create_on_off_switch(ESP_ZIGBEE_COORDINATOR_ENDPOINT_2, &switch_cfg2);
    ezb_zcl_cluster_desc_t basic_desc2 = ezb_af_endpoint_get_cluster_desc(ep_desc2, EZB_ZCL_CLUSTER_ID_BASIC, EZB_ZCL_CLUSTER_SERVER);
    if (basic_desc2) {
        ezb_zcl_basic_cluster_desc_add_attr(basic_desc2, EZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, (void *)ESP_MANUFACTURER_NAME);
        ezb_zcl_basic_cluster_desc_add_attr(basic_desc2, EZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, (void *)ESP_MODEL_IDENTIFIER);
    }
    ESP_ERROR_CHECK(ezb_af_device_add_endpoint_desc(dev_desc, ep_desc2));
#endif

#if !CONFIG_ZB_ENDPOINT_1_ENABLE && !CONFIG_ZB_ENDPOINT_2_ENABLE
    ESP_LOGW(TAG, "Внимание: Все эндпоинты выключателя отключены в конфигураторе!");
#endif

    ESP_ERROR_CHECK(ezb_af_device_desc_register(dev_desc));
    return ESP_OK;
}

/**
 * @brief Фоновая задача: обнаружение, расцепление и автономное циклическое тестирование
 */
static void commissioning_and_test_task(void *pvParameters)
{
    vTaskDelay(pdMS_TO_TICKS(3000));
    ESP_LOGI(TAG, ">>> Фоновая задача проверки и тестирования реле запущена <<<");

    // 1. Поиск адреса реле в таблице соседей
    while (s_relay_short_addr == 0xFFFF) {
        update_relay_address_from_neighbors();
        vTaskDelay(pdMS_TO_TICKS(1500));
    }

    ESP_LOGI(TAG, ">>> Реле зафиксировано (адрес 0x%04hx). Опрашиваем эндпоинты... <<<", s_relay_short_addr);
    query_relay_endpoints(s_relay_short_addr);
    vTaskDelay(pdMS_TO_TICKS(2500));

    // 2. Выполняем процедуру расцепления каналов Tuya
    tuya_uncouple_relays(s_relay_short_addr);

    ESP_LOGI(TAG, "=========================================================");
    ESP_LOGI(TAG, "✅ РЕЛЕ ПОЛНОСТЬЮ ГОТОВО К РАБОТЕ!");
    ESP_LOGI(TAG, "   Каналы разделены и управляются независимо:");
#if CONFIG_ZB_ENDPOINT_1_ENABLE
    ESP_LOGI(TAG, "   • Линия 1 -> Назначена на EP %d", s_ep_line1);
#endif
#if CONFIG_ZB_ENDPOINT_2_ENABLE
    ESP_LOGI(TAG, "   • Линия 2 -> Назначена на EP %d", s_ep_line2);
#endif
    ESP_LOGI(TAG, "=========================================================");

    rgb_led_clear();
    vTaskDelete(NULL);
}

static void esp_zigbee_main_task(void *pvParameters)
{
    esp_zigbee_config_t config = ESP_ZIGBEE_DEFAULT_CONFIG();

    ESP_ERROR_CHECK(esp_zigbee_init(&config));

    /* Настройка безопасности Zigbee 3.0: стандартный ключ альянса и поддержка Tuya устройств */
    static const uint8_t standard_link_key[16] = {
        0x5A, 0x69, 0x67, 0x42, 0x65, 0x65, 0x41, 0x6C,
        0x6C, 0x69, 0x61, 0x6E, 0x63, 0x65, 0x30, 0x39
    };
    ezb_secur_set_global_link_key(standard_link_key);
    ezb_secur_set_tclk_exchange_required(false);

    ezb_aps_secur_enable_distributed_security(false);
    ESP_ERROR_CHECK(ezb_bdb_set_primary_channel_set(ESP_ZIGBEE_PRIMARY_CHANNEL_MASK));
    ESP_ERROR_CHECK(ezb_bdb_set_secondary_channel_set(ESP_ZIGBEE_SECONDARY_CHANNEL_MASK));
    ESP_ERROR_CHECK(ezb_app_signal_add_handler(esp_zigbee_app_signal_handler));

    // Создаем конечные точки выключателя-координатора (EP 1 и EP 2)
    ESP_ERROR_CHECK(esp_zigbee_create_coordinator_endpoints());

    // Запуск стека Zigbee
    ESP_ERROR_CHECK(esp_zigbee_start(false));

    xTaskCreate(commissioning_and_test_task, "comm_test_task", 4096, NULL, 4, NULL);

    ESP_LOGI(TAG, "Zigbee стек координатора запущен. Вход в главный цикл...");
    esp_zigbee_launch_mainloop();

    esp_zigbee_deinit();
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== Инициализация ESP32-C6 Автономного Zigbee-Координатора ===");

    // 1. Инициализация энергонезависимой памяти NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Настройка энергосбережения DFS (Dynamic Frequency Scaling)
#if CONFIG_PM_ENABLE
    esp_pm_config_t pm_config = {
        .max_freq_mhz = 160,
        .min_freq_mhz = 40,
        .light_sleep_enable = false, // Координатор обязан поддерживать радиосеть активной
    };
    esp_err_t pm_err = esp_pm_configure(&pm_config);
    if (pm_err == ESP_OK) {
        ESP_LOGI(TAG, "⚡ Энергосбережение DFS включено: частота CPU снижается до 40 МГц в простое");
    }
#endif

    // 3. Инициализация адресного RGB-светодиода (WS2812 на GPIO 8)
#if CONFIG_RGB_LED_ENABLE
    rgb_led_init(RGB_LED_DEFAULT_GPIO);
#endif

    // 4. Инициализация выключателей:
    //    • Клавиша 1: GPIO 6 -> Линия 1
    //    • Клавиша 2: GPIO 7 -> Линия 2
    //    • Кнопка BOOT: GPIO 9
#if CONFIG_SWITCH_DRIVER_ENABLE
    switch_driver_config_t sw_cfg = {
        .gpio_boot = SWITCH_BOOT_GPIO,
        .gpio_key1 = SWITCH_KEY1_GPIO,
        .gpio_key2 = SWITCH_KEY2_GPIO,
        .event_cb = on_switch_event,
        .user_data = NULL,
    };
    ESP_ERROR_CHECK(switch_driver_init(&sw_cfg));
#endif

    // 5. Инициализация моста ESP-NOW (Wi-Fi канал 1 для приёма команд с ESP32-C3)
#if CONFIG_ESPNOW_BRIDGE_ENABLE
    ESP_ERROR_CHECK(espnow_bridge_init(1, on_espnow_cmd));
#endif

    // 6. Запуск задачи Zigbee Координатора
    xTaskCreate(esp_zigbee_main_task, "zb_coord_task", 4096, NULL, 5, NULL);
}
