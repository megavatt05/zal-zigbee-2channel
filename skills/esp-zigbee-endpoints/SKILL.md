---
name: esp-zigbee-endpoints
description: Полное руководство по архитектуре, созданию, динамическому автообнаружению (Active_EP_req, Simple_Desc_req), адресации (src_ep -> dst_ep), привязке (ZDO Binding) и расцеплению многоканальных Zigbee 3.0 эндпоинтов (Endpoints) в ESP-IDF (esp-zigbee-lib / ezbee API) на ESP32-C6 и ESP32-H2. Охватывает работу с многоканальными реле (Tuya TS0002/TS0004, Aqara, Sonoff), исключение неиспользуемых эндпоинтов через Kconfig (Dead-Code Elimination), фильтрацию системных эндпоинтов (Green Power EP 242) и устранение ошибок ZCL INVALID_ENDPOINT.
---

# Управление Zigbee 3.0 Эндпоинтами (Endpoints) в ESP-IDF

## 1. Архитектура эндпоинтов в Zigbee 3.0

Эндпоинт (**Endpoint, EP**) — это виртуальное логическое устройство на физическом Zigbee микроконтроллере (номера от 1 до 240):
- **EP 0**: Зарезервирован под **ZDO (Zigbee Device Objects)** — стек использует его для обнаружения сети, привязки (Binding), маршрутизации и опроса дескрипторов.
- **EP 1 – 240**: Пользовательские прикладные конечные точки (On/Off Switch, Light, Dimmer, Датчик температуры и т.д.).
- **EP 242**: Системный эндпоинт **Green Power**. Его обязательно нужно игнорировать при автообнаружении каналов реле!
- **EP 255**: Широковещательный эндпоинт (Broadcast).

В многоканальных устройствах (2-4 канальные реле и выключатели) каждый канал света представлен **отдельным эндпоинтом**:
- Линия 1 ➔ `EP 1` (Cluster 0x0006 On/Off)
- Линия 2 ➔ `EP 2` (Cluster 0x0006 On/Off)

> [!IMPORTANT]
> **Золотое правило парных эндпоинтов:**
> Если координатор управляет двумя каналами реле, он **обязан зарегистрировать у себя два локальных эндпоинта** (`Coordinator EP 1` и `Coordinator EP 2`). Если отправить команду для Линии 2 с исходящим адресом `src_ep = 1`, реле либо проигнорирует пакет, либо стек выдаст ошибку `INVALID_ENDPOINT (0x82)`.

---

## 2. Создание и регистрация локальных эндпоинтов на ESP32-C6/H2

Локальные эндпоинты создаются **до** запуска стека (`esp_zigbee_start()`) с помощью дескрипторов устройств `ezb_af`:

```c
#include "ezbee/af.h"
#include "ezbee/zcl.h"
#include "ezbee/zha/on_off_switch.h"

#define ESP_ZIGBEE_COORDINATOR_ENDPOINT_1  1
#define ESP_ZIGBEE_COORDINATOR_ENDPOINT_2  2

static esp_err_t esp_zigbee_create_coordinator_endpoints(void)
{
    // 1. Создаем общий дескриптор устройства
    ezb_af_device_desc_t dev_desc = ezb_af_create_device_desc();

#if CONFIG_ZB_ENDPOINT_1_ENABLE
    // 2. Создаем конечную точку 1 (Линия 1)
    ezb_zha_on_off_switch_config_t switch_cfg1 = EZB_ZHA_ON_OFF_SWITCH_CONFIG();
    ezb_af_ep_desc_t ep_desc1 = ezb_zha_create_on_off_switch(ESP_ZIGBEE_COORDINATOR_ENDPOINT_1, &switch_cfg1);
    
    // Добавляем обязательные атрибуты Basic кластера (0x0000)
    ezb_zcl_cluster_desc_t basic_desc1 = ezb_af_endpoint_get_cluster_desc(ep_desc1, EZB_ZCL_CLUSTER_ID_BASIC, EZB_ZCL_CLUSTER_SERVER);
    if (basic_desc1) {
        ezb_zcl_basic_cluster_desc_add_attr(basic_desc1, EZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, (void *)"Espressif");
        ezb_zcl_basic_cluster_desc_add_attr(basic_desc1, EZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, (void *)"C6_2CH_Switch");
    }
    ESP_ERROR_CHECK(ezb_af_device_add_endpoint_desc(dev_desc, ep_desc1));
#endif

#if CONFIG_ZB_ENDPOINT_2_ENABLE
    // 3. Создаем конечную точку 2 (Линия 2)
    ezb_zha_on_off_switch_config_t switch_cfg2 = EZB_ZHA_ON_OFF_SWITCH_CONFIG();
    ezb_af_ep_desc_t ep_desc2 = ezb_zha_create_on_off_switch(ESP_ZIGBEE_COORDINATOR_ENDPOINT_2, &switch_cfg2);
    
    ezb_zcl_cluster_desc_t basic_desc2 = ezb_af_endpoint_get_cluster_desc(ep_desc2, EZB_ZCL_CLUSTER_ID_BASIC, EZB_ZCL_CLUSTER_SERVER);
    if (basic_desc2) {
        ezb_zcl_basic_cluster_desc_add_attr(basic_desc2, EZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, (void *)"Espressif");
        ezb_zcl_basic_cluster_desc_add_attr(basic_desc2, EZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, (void *)"C6_2CH_Switch");
    }
    ESP_ERROR_CHECK(ezb_af_device_add_endpoint_desc(dev_desc, ep_desc2));
#endif

    // 4. Регистрируем все эндпоинты в стеке
    ESP_ERROR_CHECK(ezb_af_device_desc_register(dev_desc));
    return ESP_OK;
}
```

---

## 3. Автоматическое обнаружение эндпоинтов на удаленных реле

Никогда не хардкодьте номера эндпоинтов удаленных реле! Разные производители используют разные номера:
- **Tuya TS0002**: эндпоинты `1` и `2` (или `1` и `1` с разными DP).
- **Sonoff / eWeLink**: эндпоинты `1` и `2`.
- **Aqara QBKG**: эндпоинты `1` и `2`, либо `2` и `3`.
- **Legrand / Philips**: эндпоинты `1` и `11`.

### Двухэтапный алгоритм опроса:
1. **Запрос списка эндпоинтов (`ZDO Active_EP_req`)**
2. **Запрос кластеров каждого эндпоинта (`ZDO Simple_Desc_req`)**

```c
static uint8_t s_ep_line1 = 1;
static uint8_t s_ep_line2 = 2;

// Шаг 2: Обработка простого дескриптора и входных кластеров
static void simple_desc_cb(const ezb_zdo_simple_desc_req_result_t *result, void *user_ctx)
{
    if (result && result->rsp && result->rsp->status == 0) {
        ezb_af_simple_desc_t *sd = &result->rsp->desc;
        ESP_LOGI("ZDO_DISC", "🔍 Simple Descriptor EP %d: Profile 0x%04x, DeviceID 0x%04x, Кластеров: %d",
                 sd->ep_id, sd->app_profile_id, sd->app_device_id, sd->app_input_cluster_count);

        for (int i = 0; i < sd->app_input_cluster_count; i++) {
            uint16_t c_id = sd->app_cluster_list[i];
            if (c_id == 0x0006) { // On/Off Cluster
                ESP_LOGI("ZDO_DISC", "  🎯 Найден кластер On/Off на эндпоинте %d!", sd->ep_id);
            }
        }
    }
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

// Шаг 1: Получение списка всех активных эндпоинтов реле
static void active_ep_cb(const ezb_zdo_active_ep_req_result_t *result, void *user_ctx)
{
    if (!result || !result->rsp || result->rsp->status != 0) return;

    ESP_LOGI("ZDO_DISC", "🎉 Найдено активных эндпоинтов: %d", result->rsp->active_ep_count);

    uint8_t found_relays = 0;
    for (int i = 0; i < result->rsp->active_ep_count; i++) {
        uint8_t ep = result->rsp->active_ep_list[i];

        // Игнорируем Green Power эндпоинт 242
        if (ep == 242) continue;

        if (found_relays == 0) s_ep_line1 = ep;
        else if (found_relays == 1) s_ep_line2 = ep;
        found_relays++;

        // Опрашиваем кластеры каждого эндпоинта
        query_simple_desc(result->rsp->nwk_addr_of_interest, ep);
    }

    ESP_LOGI("ZDO_DISC", ">>> ИТОГ: Линия 1 -> EP %d | Линия 2 -> EP %d <<<", s_ep_line1, s_ep_line2);
}

void query_relay_endpoints(uint16_t short_addr)
{
    ezb_zdo_active_ep_req_t req = {
        .dst_nwk_addr = short_addr,
        .field = { .nwk_addr_of_interest = short_addr },
        .cb = active_ep_cb,
    };
    esp_zigbee_lock_acquire(portMAX_DELAY);
    ezb_zdo_active_ep_req(&req);
    esp_zigbee_lock_release();
}
```

---

## 4. Корректная адресация команд ZCL (Source EP ➔ Destination EP)

При отправке ZCL команд On, Off или Toggle необходимо строго выставлять соответствующий `src_ep`:

```c
void send_relay_cmd(uint8_t line_endpoint, uint8_t action)
{
    // Для Линии 2 используем Coordinator EP 2, для Линии 1 - Coordinator EP 1
    uint8_t src_ep = (line_endpoint == s_ep_line2) ? ESP_ZIGBEE_COORDINATOR_ENDPOINT_2 : ESP_ZIGBEE_COORDINATOR_ENDPOINT_1;

    ezb_zcl_on_off_cmd_t cmd_req = {
        .cmd_ctrl = {
            .dst_addr = {
                .addr_mode = EZB_ADDR_MODE_SHORT,
                .u.short_addr = s_relay_short_addr,
            },
            .src_ep = src_ep,          // Локальный эндпоинт координатора
            .dst_ep = line_endpoint,   // Целевой эндпоинт на реле (EP 1 или EP 2)
            .dis_default_rsp = true,
        },
    };

    esp_zigbee_lock_acquire(portMAX_DELAY);
    if (action == 0) {
        ezb_zcl_on_off_off_cmd_req(&cmd_req);
    } else if (action == 1) {
        ezb_zcl_on_off_on_cmd_req(&cmd_req);
    } else {
        ezb_zcl_on_off_toggle_cmd_req(&cmd_req);
    }
    esp_zigbee_lock_release();
}
```

---

## 5. Привязка эндпоинтов через ZDO Binding

Связывание (Binding) сообщает реле, куда отправлять подтверждения и отчёты о состоянии:

```c
void bind_endpoint_cluster(uint16_t short_addr, uint8_t dev_ep, uint8_t coord_ep)
{
    ezb_extaddr_t dev_ieee, coord_ieee;
    if (ezb_address_extended_by_short(short_addr, &dev_ieee) != EZB_ERR_NONE) return;
    if (ezb_address_extended_by_short(0x0000, &coord_ieee) != EZB_ERR_NONE) return;

    ezb_zdo_bind_req_t req = {
        .dst_nwk_addr = short_addr,
        .field = {
            .src_addr = dev_ieee,       // 64-bit IEEE адрес реле
            .src_ep = dev_ep,           // Эндпоинт на реле
            .cluster_id = 0x0006,       // Кластер On/Off
            .dst_addr_mode = 0x03,      // 64-bit расширенный адрес
            .dst_addr = { .extended_addr = coord_ieee }, // 64-bit адрес координатора
            .dst_ep = coord_ep,         // Эндпоинт на координаторе
        },
        .cb = NULL,
    };

    esp_zigbee_lock_acquire(portMAX_DELAY);
    ezb_zdo_bind_req(&req);
    esp_zigbee_lock_release();
}
```

---

## 6. Меню Kconfig и вырезание неиспользуемых эндпоинтов

Если выключатель используется как одноклавишный, поддержка `EP 2` вырезается на этапе компиляции, экономя Flash и память RAM:

```kconfig
menu "Управление конечными точками Zigbee (Endpoints)"

    config ZB_ENDPOINT_1_ENABLE
        bool "Включить Конечную Точку 1 (Линия 1 / EP 1)"
        default y
        help
            Включает создание и регистрацию конечной точки EP 1 (кластер On/Off 0x0006).

    config ZB_ENDPOINT_2_ENABLE
        bool "Включить Конечную Точку 2 (Линия 2 / EP 2)"
        default y
        help
            Включает создание и регистрацию конечной точки EP 2 (кластер On/Off 0x0006).

endmenu
```

**Безопасные `#ifndef` дефолты в C:**
```c
#ifndef CONFIG_ZB_ENDPOINT_1_ENABLE
#define CONFIG_ZB_ENDPOINT_1_ENABLE 1
#endif
#ifndef CONFIG_ZB_ENDPOINT_2_ENABLE
#define CONFIG_ZB_ENDPOINT_2_ENABLE 1
#endif
```

---

## 7. Чеклист частых ошибок с эндпоинтами

| Ошибка | Причина | Решение |
| :--- | :--- | :--- |
| `INVALID_ENDPOINT (0x82)` | Отправка с незарегистрированного `src_ep` координатора | Зарегистрировать эндпоинты в `ezb_af_device_desc_register` до `esp_zigbee_start()` |
| Реле переключает сразу оба канала | Заводское спаривание каналов Tuya | Выполнить сиквенс расцепления: запись `0xFFDE=0x0D`, пауза 2.6с, разблокировка `0xF0` |
| Ошибка ZDO Simple_Desc_req | Запрос кластеров системного Green Power эндпоинта 242 | Добавить фильтр: `if (ep == 242) continue;` |
| Реле не отвечает на команды | Неверный целевой `dst_ep` | Запустить автообнаружение `ezb_zdo_active_ep_req()` и сохранить `active_ep_list` |
