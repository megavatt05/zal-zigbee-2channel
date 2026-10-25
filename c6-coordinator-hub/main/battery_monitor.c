/*
 * SPDX-FileCopyrightText: 2026 Antigravity
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Мониторинг напряжения батареи
 */

#include "sdkconfig.h"

#if CONFIG_BATTERY_MONITOR_ENABLE

#include "battery_monitor.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "BATTERY_MONITOR";

static battery_monitor_config_t s_cfg = {
    .adc_channel = -1,
    .divider_r1_kohm = 1000,
    .divider_r2_kohm = 1000,
    .min_voltage_mv = 2600,
    .max_voltage_mv = 3000,
};

static adc_oneshot_unit_handle_t s_adc_handle = NULL;
static adc_cali_handle_t s_cali_handle = NULL;

esp_err_t battery_monitor_init(const battery_monitor_config_t *config)
{
    if (config) {
        s_cfg = *config;
    }

    if (s_cfg.adc_channel >= 0) {
        adc_oneshot_unit_init_cfg_t init_config = {
            .unit_id = ADC_UNIT_1,
        };
        ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &s_adc_handle));

        adc_oneshot_chan_cfg_t chan_config = {
            .bitwidth = ADC_BITWIDTH_DEFAULT,
            .atten = ADC_ATTEN_DB_12,
        };
        ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, (adc_channel_t)s_cfg.adc_channel, &chan_config));

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = ADC_UNIT_1,
            .chan = (adc_channel_t)s_cfg.adc_channel,
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        adc_cali_create_scheme_curve_fitting(&cali_config, &s_cali_handle);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = ADC_UNIT_1,
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        adc_cali_create_scheme_line_fitting(&cali_config, &s_cali_handle);
#endif
        ESP_LOGI(TAG, "ADC инициализирован на канале ADC1_%d", s_cfg.adc_channel);
    } else {
        ESP_LOGI(TAG, "ADC мониторинг отключен (эмуляция 3.0В номинала)");
    }

    return ESP_OK;
}

uint32_t battery_monitor_get_voltage_mv(void)
{
    if (!s_adc_handle || s_cfg.adc_channel < 0) {
        return 3000; // Номинальные 3.0В по умолчанию
    }

    int raw_val = 0;
    int voltage_mv = 0;
    if (adc_oneshot_read(s_adc_handle, (adc_channel_t)s_cfg.adc_channel, &raw_val) == ESP_OK) {
        if (s_cali_handle) {
            adc_cali_raw_to_voltage(s_cali_handle, raw_val, &voltage_mv);
        } else {
            voltage_mv = (raw_val * 3300) / 4095;
        }

        // Пересчет с учетом резистивного делителя
        if (s_cfg.divider_r2_kohm > 0) {
            voltage_mv = voltage_mv * (s_cfg.divider_r1_kohm + s_cfg.divider_r2_kohm) / s_cfg.divider_r2_kohm;
        }
        return (uint32_t)voltage_mv;
    }

    return 3000;
}

uint8_t battery_monitor_get_zb_voltage(void)
{
    uint32_t mv = battery_monitor_get_voltage_mv();
    uint32_t units_100mv = mv / 100;
    if (units_100mv > 255) {
        units_100mv = 255;
    }
    return (uint8_t)units_100mv;
}

uint8_t battery_monitor_get_percentage(void)
{
    uint32_t mv = battery_monitor_get_voltage_mv();
    if (mv >= (uint32_t)s_cfg.max_voltage_mv) {
        return 100;
    }
    if (mv <= (uint32_t)s_cfg.min_voltage_mv) {
        return 0;
    }
    return (uint8_t)(((mv - s_cfg.min_voltage_mv) * 100) / (s_cfg.max_voltage_mv - s_cfg.min_voltage_mv));
}

uint8_t battery_monitor_get_zb_percentage_remaining(void)
{
    // Zigbee спецификация: единицы 0.5% (200 = 100%)
    uint8_t pct = battery_monitor_get_percentage();
    return (uint8_t)(pct * 2);
}

#else // !CONFIG_BATTERY_MONITOR_ENABLE

#include "battery_monitor.h"

esp_err_t battery_monitor_init(const battery_monitor_config_t *config) { (void)config; return ESP_OK; }
uint32_t battery_monitor_get_voltage_mv(void) { return 3300; }
uint8_t battery_monitor_get_zb_voltage(void) { return 33; }
uint8_t battery_monitor_get_percentage(void) { return 100; }
uint8_t battery_monitor_get_zb_percentage_remaining(void) { return 200; }

#endif // CONFIG_BATTERY_MONITOR_ENABLE

