---
name: esp-zigbee-wifi-bridge
description: Архитектура, радио-протоколы и аппаратные драйверы для автономного моста «ESP-NOW ➔ Zigbee 3.0» на ESP32-C6 / ESP32-H2 и сверхнизкопотребляющих батарейных пультов на ESP32-C3 / ESP32. Включает решение TDM Coexistence радио (Wi-Fi ch 1 / 802.15.4 ch 15), пачечную передачу Burst 80 мс, дедупликацию, 5-шаговый сиквенс расцепления спаренных каналов Tuya TS0002 (Relay Uncoupling), Deep Sleep ~5 мкА с GPIO Wakeup в ESP-IDF 6.1, замер RSSI, Kconfig dead-code elimination. Использовать при разработке Zigbee координаторов, беспроводных выключателей, связи между платами ESP по Wi-Fi/ESP-NOW.
---

# Мост «ESP-NOW ➔ Zigbee 3.0» и батарейные пульты на ESP32

## 1. Архитектурная концепция

### Зачем нужен гибридный мост:
- **ESP32-C3**: Идеален для автономных батарейных выключателей (низкая цена, Deep Sleep ~5 мкА, сверхбыстрый старт), но **не имеет радио 802.15.4** (нет аппаратной поддержки Zigbee/Thread).
- **ESP32-C6**: Имеет мультипротокольный радиотракт 2.4 ГГц (Wi-Fi 6 + IEEE 802.15.4 Zigbee 3.0 / Thread). Подключается к сети 230 В / 5 В и работает как постоянно активный **Zigbee Координатор** и **ESP-NOW приёмник**.
- **Реле Tuya (TS0002)**: Стандартное исполнительное реле Zigbee 3.0, питающееся от сети 230 В.

```text
┌───────────────────────────────┐        ESP-NOW        ┌───────────────────────────────┐        Zigbee 3.0 ZCL        ┌───────────────────────────────┐
│     ESP32-C3 Пульт            │ ────────────────────► │     ESP32-C6 Шлюз             │ ───────────────────────────► │  2-Канальное Реле Tuya TS0002 │
│  • Батарейка CR2032 (~5 мкА)  │  Burst 80мс, +21 dBm  │  • Питание 5V USB / 230V      │     IEEE 802.15.4 ch 15      │  • Линия 1 (EP 1): Канал 1    │
│  • Deep Sleep + GPIO Wakeup   │  Wi-Fi AP Канал 1     │  • Zigbee Координатор         │     On/Off Cluster (0x0006)  │  • Линия 2 (EP 2): Канал 2    │
│  • Клавиша 1: GPIO 2 ➔ GND    │  Unicast + Broadcast  │  • ESP-NOW Мост (Wi-Fi AP)    │                              └───────────────────────────────┘
│  • Клавиша 2: GPIO 3 ➔ GND    │                       │  • Coexistence TDM Радио      │
└───────────────────────────────┘                       │  • Дедупликация повторов 350мс│
                                                        │  • Расцепление каналов Tuya   │
                                                        └───────────────────────────────┘
```

---

## 2. Аппаратный Coexistence TDM на ESP32-C6: ловушка и решение

### В чём проблема:
ESP32-C6 использует **один физический радиотракт 2.4 ГГц** для Wi-Fi и Zigbee. Механизм Coexistence делит эфир по времени (**TDM — Time Division Multiplexing**):
- Радио слушает Zigbee (канал 15) в течение 20–50 мс (опрос устройств, keepalive, ZDO beacon).
- Затем переключается на Wi-Fi (канал 1) в течение следующего временного окна.
- **Следствие:** если батарейный пульт шлёт короткую одиночную посылку (4–6 мс), она попадает в окно Zigbee и теряется. Кнопки «срабатывают через раз».

### Рецепт 100% надёжности связи:
1. **На стороне пульта (ESP32-C3): Пачечная передача Burst 80 мс**
   Вместо одиночного пакета пульт шлёт серию из **8 импульсов с шагом 10 мс** (суммарно ровно 80 мс в эфире) на максимальной аппаратной мощности `esp_wifi_set_max_tx_power(84)` (+21 dBm). Серия отправляется одновременно на Broadcast (`FF:FF:FF:FF:FF:FF`) и Unicast MAC точки доступа C6. 80 мс гарантированно перекрывают TDM-окно Zigbee.
2. **На стороне шлюза (ESP32-C6): Дедупликация пакетов пачки**
   Хаб отслеживает номер пакета (`seq`) и временную метку:
   ```c
   static uint8_t s_last_pkt_seq = 0;
   static uint32_t s_last_pkt_tick = 0;

   static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int data_len)
   {
       if (!recv_info || !data || data_len < (int)sizeof(espnow_switch_pkt_t)) return;
       const espnow_switch_pkt_t *pkt = (const espnow_switch_pkt_t *)data;
       if (pkt->magic != ESPNOW_SWITCH_MAGIC) return;

       // Замер RSSI (dBm)
       int8_t rssi = recv_info->rx_ctrl ? recv_info->rx_ctrl->rssi : 0;
       ESP_LOGW("RADIO_RSSI", "📶 [RSSI C3->C6]: %d dBm | Линия: %d | Seq: %d", rssi, pkt->line_id, pkt->seq);

       // Дедупликация: игнорируем дубли одной пачки в течение 350 мс
       uint32_t now = (uint32_t)xTaskGetTickCount();
       if (pkt->seq == s_last_pkt_seq && (now - s_last_pkt_tick) < pdMS_TO_TICKS(350)) {
           return;
       }
       s_last_pkt_seq = pkt->seq;
       s_last_pkt_tick = now;

       // Мгновенное исполнение команды на реле
       xQueueSend(s_espnow_queue, pkt, 0);
   }
   ```
   **Результат:** реле щёлкает мгновенно по первому пойманному пакету, а повторные 7 пакетов отбрасываются (нет двойных щелчков).

---

## 3. Расцепление спаренных каналов Tuya TS0002 (Relay Uncoupling)

Двухканальные реле Tuya (`_TZ3000_...`) аппаратно спарены на заводе: изменение состояния первого канала синхронно включает второй. Для их разделения в координаторе выполняется строгий 5-шаговый сиквенс:

```c
void tuya_uncouple_relays(uint16_t short_addr)
{
    // Шаг 1: Чтение базового атрибута производителя 0xFF
    read_tuya_attr_0xff(short_addr, 1);
    vTaskDelay(pdMS_TO_TICKS(300));

    // Шаг 2: Запись служебного атрибута расцепления 0xFFDE = 0x0D (тип 0x21 uint16, значение 0x0001)
    write_tuya_attr_0xffde(short_addr, 1, 0x0001);

    // Шаг 3: Технологическая пауза 2.6 сек на внутреннюю перезагрузку реле
    vTaskDelay(pdMS_TO_TICKS(2600));

    // Шаг 4: Отправка ZCL команды разблокировки 0xF0 на EP 1
    send_tuya_unlock_cmd_0xf0(short_addr, 1);
    vTaskDelay(pdMS_TO_TICKS(500));

    // Шаг 5: Раздельная ZDO привязка (Bind) кластера On/Off (0x0006)
    bind_endpoint_cluster(short_addr, 1, ESP_ZIGBEE_COORDINATOR_ENDPOINT_1); // Линия 1 -> ZC EP 1
    vTaskDelay(pdMS_TO_TICKS(250));
    bind_endpoint_cluster(short_addr, 2, ESP_ZIGBEE_COORDINATOR_ENDPOINT_2); // Линия 2 -> ZC EP 2
}
```

---

## 4. Батарейный пульт ESP32-C3: Deep Sleep ~5 мкА в ESP-IDF 6.1

### Правильная настройка сна и пробуждения:
В ESP-IDF 6.1 устаревшие вызовы заменены. Для пробуждения из глубокого сна по низкому уровню (замыкание на GND) используется:

```c
#include "esp_sleep.h"
#include "driver/gpio.h"

#define SWITCH_KEY1_GPIO  GPIO_NUM_2
#define SWITCH_KEY2_GPIO  GPIO_NUM_3

// Счётчик пакетов сохраняется в энергонезависимой памяти RTC
static RTC_DATA_ATTR uint8_t s_pkt_seq = 0;

void app_main(void)
{
    uint64_t pin_mask = (1ULL << SWITCH_KEY1_GPIO) | (1ULL << SWITCH_KEY2_GPIO);

    // 1. Настройка входных пинов
    gpio_config_t io_conf = {
        .pin_bit_mask = pin_mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // 2. Определение источника пробуждения
    uint8_t target_line = 0;
    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_GPIO) {
        uint64_t mask = esp_sleep_get_gpio_wakeup_status();
        if (mask & (1ULL << SWITCH_KEY1_GPIO)) target_line = 1;
        if (mask & (1ULL << SWITCH_KEY2_GPIO)) target_line = 2;
    }
    // Резервная проверка уровней пинов
    if (target_line == 0) {
        if (gpio_get_level(SWITCH_KEY1_GPIO) == 0) target_line = 1;
        else if (gpio_get_level(SWITCH_KEY2_GPIO) == 0) target_line = 2;
    }

    // 3. Отправка команды
    if (target_line != 0) {
        init_wifi_and_espnow();
        send_burst_packet(target_line, 8, 10); // 8 пакетов по 10 мс = 80 мс
        deinit_wifi_and_espnow();

        // Ожидание отпускания клавиши (до 300 мс) + антидребезг 30 мс
        int timeout = 30;
        while ((gpio_get_level(SWITCH_KEY1_GPIO) == 0 || gpio_get_level(SWITCH_KEY2_GPIO) == 0) && timeout-- > 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }

    // 4. Фиксация аппаратных подтяжек во время глубокого сна (GPIO Hold)
    gpio_pullup_en(SWITCH_KEY1_GPIO);
    gpio_hold_en(SWITCH_KEY1_GPIO);
    gpio_pullup_en(SWITCH_KEY2_GPIO);
    gpio_hold_en(SWITCH_KEY2_GPIO);
    gpio_deep_sleep_hold_en();

    // 5. Разрешение пробуждения из Deep Sleep (ESP-IDF 6.1 API)
    ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup_on_hp_periph_powerdown(pin_mask, ESP_GPIO_WAKEUP_GPIO_LOW));

    // Уход в сон: ток падает до ~5 мкА
    esp_deep_sleep_start();
}
```

### Важный нюанс встроенного USB:
На ESP32-C3 встроенный контроллер USB-Serial/JTAG обесточивается при уходе в Deep Sleep. В Windows виртуальный COM-порт отключается — это нормальное аппаратное поведение спящего чипа.

---

## 5. Измерение качества радиосигнала (RSSI)

В ESP-NOW уровень сигнала считывается напрямую из заголовка кадра без накладных расходов:
```c
int8_t rssi = recv_info->rx_ctrl ? recv_info->rx_ctrl->rssi : 0;
```

**Шкала качества связи ESP-NOW:**
- **`-30 ... -55 dBm`**: Идеальный сигнал (платы в пределах 1–2 метров).
- **`-56 ... -72 dBm`**: Отличный сигнал (в пределах комнаты / прямая видимость).
- **`-73 ... -82 dBm`**: Уверенная связь (через 1–2 железобетонные стены).
- **`< -88 dBm`**: Граница чувствительности (требуется увеличить число повторов Burst).

---

## 6. Kconfig и Dead-Code Elimination (Оптимизация Flash)

Для уменьшения размера бинарного файла и экономии ОЗУ:
1. **Безопасные дефолты препроцессора:**
   Если Kconfig опции ещё не сгенерированы в `sdkconfig.h`, объявлять защитные fallback-макросы:
   ```c
   #ifndef CONFIG_ZB_ENDPOINT_1_ENABLE
   #define CONFIG_ZB_ENDPOINT_1_ENABLE 1
   #endif
   ```
2. **Вырезание строк логирования:**
   ```c
   #if !CONFIG_ZB_LOG_ENABLE
   #undef ESP_LOGI
   #define ESP_LOGI(tag, fmt, ...) ((void)0)
   #endif
   ```
   Удаляет сотни форматированных строк из секции `.rodata`, освобождая **более 38 КБайт Flash**.
3. **No-op Stubs:** при отключении модуля (например, RGB LED или локальных кнопок) заменять функции пустыми заглушками, позволяя линковщику (`--gc-sections`) полностью исключить драйвер RMT / GPIO из сборки.

---

## 7. Команды сборки и прошивки (Справочник)

```powershell
# Сборка через официальный wrapper на этой машине:
powershell -NoProfile -File C:\Users\megav\.espressif\hermes\idf.ps1 -Idf v6.1 -Dir <путь_к_проекту> -Task "build"

# Прошивка:
powershell -NoProfile -File C:\Users\megav\.espressif\hermes\idf.ps1 -Idf v6.1 -Dir <путь_к_проекту> -Task "-p COM36 flash"

# Мониторинг логов без сброса:
powershell -NoProfile -File C:\Users\megav\.espressif\hermes\serial-log.ps1 -Port COM36 -Seconds 10
```
