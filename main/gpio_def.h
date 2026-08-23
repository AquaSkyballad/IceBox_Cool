#pragma once

#include "driver/gpio.h"
#include "hal/adc_types.h"

/* 定义GPIO引脚 */

// ADC - NTC 温度传感器 (10kΩ NTC, B=3950)
#define AIR_ADC_GPIO   GPIO_NUM_4   /* AIR_ADC:  箱内空气温度, 33kΩ 上拉         */
#define COLD_ADC_GPIO  GPIO_NUM_5   /* COLD_ADC: TEC 冷端温度, 33kΩ 上拉 (温控反馈) */
#define HOT_ADC_GPIO   GPIO_NUM_6   /* HOT_ADC:  TEC 热端温度, 3.3kΩ 上拉 (安全保护) */
#define G_ADC_GPIO     GPIO_NUM_7   /* G_ADC:    TEC- 电压检测, 100kΩ/15kΩ/220kΩ 正偏置网络 */

#define AIR_ADC_CHANNEL   ADC_CHANNEL_3
#define COLD_ADC_CHANNEL  ADC_CHANNEL_4
#define HOT_ADC_CHANNEL   ADC_CHANNEL_5
#define G_ADC_CHANNEL     ADC_CHANNEL_6

// LED
#define LED_SAT_GPIO   GPIO_NUM_8   /* LED_SAT:  状态指示 LED (极性待上板确认)    */

// TFT (ST7789V2, SPI2/FSPI 单向写)
#define TFT_BLK_GPIO   GPIO_NUM_9   /* TFT_BLK:  背光 PWM 正相(高=亮) ~1kHz       */
#define TFT_CS_GPIO    GPIO_NUM_10  /* TFT_CS:   片选, 低有效                     */
#define TFT_MOSI_GPIO  GPIO_NUM_11  /* TFT_MOSI: FSPID, 单向写数据               */
#define TFT_SCK_GPIO   GPIO_NUM_12  /* TFT_CLK:  FSPICLK                        */
#define TFT_DC_GPIO    GPIO_NUM_13  /* TFT_DC:   数据/命令选择                    */
#define TFT_RST_GPIO   GPIO_NUM_14  /* TFT_RST:  硬复位, 低有效                  */

// 三向拨轮 (外部 10kΩ 上拉, 低有效)
#define KEY1_GPIO      GPIO_NUM_15  /* KEY1: 语义待最终丝印确认                   */
#define KEY2_GPIO      GPIO_NUM_16  /* KEY2: 语义待最终丝印确认                   */
#define KEY3_GPIO      GPIO_NUM_17  /* KEY3: 语义待最终丝印确认                   */

// USB (原生 USB-Serial-JTAG)
#define USB_DN_GPIO    GPIO_NUM_19  /* USB D-, 90Ω 差分                        */
#define USB_DP_GPIO    GPIO_NUM_20  /* USB D+, 90Ω 差分                        */

// I2C (INA226 总线)
#define I2C_SCL_GPIO   GPIO_NUM_21  /* I2C_SCL: 开漏, 4.7kΩ 上拉                */
#define I2C_SDA_GPIO   GPIO_NUM_47  /* I2C_SDA: 开漏, 4.7kΩ 上拉                */

// TEC 功率控制
#define TEC_PWM_GPIO   GPIO_NUM_38  /* TEC_PWM: 100kHz, 高=开通, UCC27517 同相   */

// FAN1 (4-pin PWM, ~23kHz 反相)
#define FAN1_PWM_GPIO  GPIO_NUM_39  /* FAN1_PWM:  MCU高→NMOS导通→风扇线低        */
#define FAN1_TACH_GPIO GPIO_NUM_40  /* FAN1_TACH: 开漏, 10kΩ 上拉, 2脉冲/转     */

// FAN2 (4-pin PWM, ~23kHz 反相)
#define FAN2_PWM_GPIO  GPIO_NUM_41  /* FAN2_PWM:  MCU高→NMOS导通→风扇线低        */
#define FAN2_TACH_GPIO GPIO_NUM_42  /* FAN2_TACH: 开漏, 10kΩ 上拉, 2脉冲/转     */

// INA226 报警中断
#define INA_ALT_GPIO   GPIO_NUM_48  /* ALT: INA226 开漏报警, 上拉到3V3, 低有效   */
