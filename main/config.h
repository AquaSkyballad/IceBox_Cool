/**
 * @file    config.h
 * @brief   系统配置参数和硬件常量定义
 *
 * 集中管理所有可配置参数,提供默认值用于故障恢复
 */

#pragma once

#include <math.h>
#include <stdint.h>

#include "sdkconfig.h"
#include "board_config.h"

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

/* Temperature-loop defaults exposed through Kconfig (x1000 -> float). */
#define PID_KP_DEFAULT                   ((float)CONFIG_ICEBOX_CONTROL_TEMP_PID_KP_X1000 / 1000.0f)
#define PID_KI_DEFAULT                   ((float)CONFIG_ICEBOX_CONTROL_TEMP_PID_KI_X1000 / 1000.0f)
#define PID_KD_DEFAULT                   ((float)CONFIG_ICEBOX_CONTROL_TEMP_PID_KD_X1000 / 1000.0f)
#define TARGET_TEMP_DEFAULT              CONFIG_ICEBOX_TARGET_TEMP_DEFAULT_C
#define ICEBOX_TARGET_TEMP_MIN_C         (-10.0f)
#define ICEBOX_TARGET_TEMP_MAX_C         (40.0f)

/* TEC current boundaries. Keep the three levels distinct:
 * - normal control must never request more than the software maximum;
 * - the safety task trips on a measured value at or above the software
 *   over-current trip point;
 * - INA226 SOL/ALT remains the final hardware over-current limit. */
#define TEC_CURRENT_SOFTWARE_MAX_A        5.5f
#define TEC_CURRENT_SAFETY_TRIP_A         5.8f
#define TEC_CURRENT_ALT_LIMIT_A           6.0f

/* Safety policy. Fixed values stay in config.h; selected timing/temperature
 * defaults below are supplied by Kconfig and adapted here. */
#define SAFETY_DC_START_MIN_V                  11.0f
#define SAFETY_DC_START_MAX_V                  22.0f
#define SAFETY_DC_STABLE_TIME_MS               CONFIG_ICEBOX_SAFETY_DC_STABLE_TIME_MS
#define SAFETY_NO_DC_MAX_V                     1.0f
#define SAFETY_HARD_OVERVOLTAGE_V              22.0f
#define SAFETY_STARTUP_MAX_CURRENT_A           0.10f
#define SAFETY_STARTUP_PWM_OFF_FAULT_CURRENT_A 2.50f
#define SAFETY_STARTUP_CHECK_TIMEOUT_MS        500U
#define SAFETY_HOT_START_MAX_TEMP_C            ((float)CONFIG_ICEBOX_SAFETY_HOT_MAX_TEMP_C)
#define SAFETY_FAN_STARTUP_MIN_PULSES          1U
#define SAFETY_FAN_STARTUP_WINDOW_MS           500U
#define SAFETY_FAN_RUNTIME_WINDOW_MS           500U
#define SAFETY_RECOVERY_NORMAL_RUNTIME_MS      (30U * 60U * 1000U)
#define SAFETY_INA_STALE_TIMEOUT_MS            120U
#define SAFETY_INA_RECONFIGURE_ATTEMPTS        2U
#define SAFETY_ADC_UPDATE_PERIOD_MS            20U
#define SAFETY_HOT_STALE_FAILURE_LIMIT         3U
#define SAFETY_LOW_VOLTAGE_10V5_SAMPLES        16U
#define SAFETY_LOW_VOLTAGE_9V5_SAMPLES         3U

#ifdef __cplusplus
}
#endif
