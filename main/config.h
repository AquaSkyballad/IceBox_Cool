/**
 * @file    config.h
 * @brief   系统配置参数和硬件常量定义
 *
 * 集中管理所有可配置参数,提供默认值用于故障恢复
 */

#pragma once

#include <math.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ADC采样 */
#define ADC_SENSOR_SAMPLE_COUNT          16U
#define ADC_SENSOR_MAX_READ_ATTEMPTS     20U
#define ADC_SENSOR_BOARD_SUPPLY_MV       3300

/* 10kΩ B3950 NTC，基准点为25°C。 */
#define ADC_NTC_NOMINAL_RESISTANCE_OHM   10000.0f
#define ADC_NTC_BETA_K                   3950.0f
#define ADC_NTC_REFERENCE_TEMP_K         298.15f
#define ADC_AIR_PULLUP_RESISTANCE_OHM    33000.0f
#define ADC_COLD_PULLUP_RESISTANCE_OHM   33000.0f
#define ADC_HOT_PULLUP_RESISTANCE_OHM    3300.0f

/* TEC-N偏置分压网络：V_N = (V_ADC - offset) / scale。 */
#define ADC_TEC_N_OFFSET_V               0.18469f
#define ADC_TEC_N_SCALE                  0.12313f

/*
 * 2026-08-24裸板实测：三路NTC悬空均为raw=4095、校准电压=3170 mV。
 * 使用校准电压而不是raw判断开路，门限取3100 mV，保留70 mV余量。
 * 短路和正常温度边界仍等待连接传感器后的样机数据。
 */
#define ADC_NTC_OPEN_THRESHOLD_MV        3100
#define ADC_AIR_VALID_MV_MIN             0
#define ADC_AIR_VALID_MV_MAX             ADC_NTC_OPEN_THRESHOLD_MV
#define ADC_COLD_VALID_MV_MIN            0
#define ADC_COLD_VALID_MV_MAX            ADC_NTC_OPEN_THRESHOLD_MV
#define ADC_HOT_VALID_MV_MIN             0
#define ADC_HOT_VALID_MV_MAX             ADC_NTC_OPEN_THRESHOLD_MV
#define ADC_TEC_N_VALID_MV_MIN            0
#define ADC_TEC_N_VALID_MV_MAX            ADC_SENSOR_BOARD_SUPPLY_MV

#define ADC_AIR_VALID_TEMP_C_MIN         -20
#define ADC_AIR_VALID_TEMP_C_MAX         60
#define ADC_COLD_VALID_TEMP_C_MIN        -20
#define ADC_COLD_VALID_TEMP_C_MAX        60
#define ADC_HOT_VALID_TEMP_C_MIN         -20
#define ADC_HOT_VALID_TEMP_C_MAX         120
#define ADC_TEC_N_VALID_VOLTAGE_V_MIN    -2
#define ADC_TEC_N_VALID_VOLTAGE_V_MAX    22

/* Safety policy. Keep these as project macros so they can be migrated to
 * Kconfig without changing the safety module's call sites. */
#define SAFETY_DC_START_MIN_V                  11.0f
#define SAFETY_DC_START_MAX_V                  22.0f
#define SAFETY_DC_STABLE_TIME_MS               500U
#define SAFETY_NO_DC_MAX_V                     1.0f
#define SAFETY_HARD_OVERVOLTAGE_V              22.0f
#define SAFETY_HARD_CURRENT_A                  5.5f
#define SAFETY_STARTUP_MAX_CURRENT_A           0.10f
#define SAFETY_STARTUP_PWM_OFF_FAULT_CURRENT_A 2.50f
#define SAFETY_STARTUP_CHECK_TIMEOUT_MS        500U
#define SAFETY_TEC_CONNECT_VALID_SAMPLES       3U
#define SAFETY_TEC_CLOSED_MAX_DIFF_V           1.0f
#define SAFETY_TEC_OPEN_MAX_ABS_V              1.0f
#define SAFETY_HOT_START_MAX_TEMP_C            60.0f
#define SAFETY_FAN_STARTUP_MIN_PULSES          1U
#define SAFETY_FAN_STARTUP_WINDOW_MS           500U
#define SAFETY_FAN_RUNTIME_WINDOW_MS           500U
#define SAFETY_INA_STALE_TIMEOUT_MS            120U
#define SAFETY_INA_RECONFIGURE_ATTEMPTS        2U
#define SAFETY_ADC_UPDATE_PERIOD_MS            20U
#define SAFETY_HOT_STALE_FAILURE_LIMIT         3U
#define SAFETY_LOW_VOLTAGE_10V5_SAMPLES        16U
#define SAFETY_LOW_VOLTAGE_9V5_SAMPLES         3U

#ifdef __cplusplus
}
#endif
