#include "driver/gpio.h"
/* 定义GPIO引脚 — 基于 circuit.md 实测网表 */

// NCP81239
#define P_EN_GPIO   GPIO_NUM_21   /* P_EN: 使能功率级   */
#define P_INT_GPIO  GPIO_NUM_42   /* INT:  中断输入     */
#define P_SCL_GPIO  GPIO_NUM_18   /* SCL:  I2C 时钟     */
#define P_SDA_GPIO  GPIO_NUM_17   /* SDA:  I2C 数据     */

// NTC
#define NTCN_GPIO   GPIO_NUM_5    /* NTC_N:  上拉 3.3k (通用) */
#define NTCL1_GPIO  GPIO_NUM_6    /* NTC_L1: 上拉 33k  (低温段)       */
#define NTCL2_GPIO  GPIO_NUM_7    /* NTC_L2: 上拉 33k  (低温段)       */

// thumbwheel (五向拨轮 QS-302, active-low)
#define THUMBWHEEL_KEY1_GPIO  GPIO_NUM_15  /* U6.1: 左/右 */
#define THUMBWHEEL_KEY2_GPIO  GPIO_NUM_48  /* U6.T: 按下  */
#define THUMBWHEEL_KEY3_GPIO  GPIO_NUM_16  /* U6.2: 右/左 */

// USB (原生 USB-Serial-JTAG)
#define USB_DP_GPIO  GPIO_NUM_20   /* D+ */
#define USB_DN_GPIO  GPIO_NUM_19   /* D- */

// TFT (ST7789V2, SPI2 单向写, FSPI)
#define TFT_RST_GPIO  GPIO_NUM_10   /* D_RST: 硬复位                           */
#define TFT_MOSI_GPIO GPIO_NUM_11   /* D_MOSI: FSPID                           */
#define TFT_SCK_GPIO  GPIO_NUM_12   /* D_CLK:  FSPICLK                         */
#define TFT_CS_GPIO   GPIO_NUM_13   /* D_CS:   FSPICS0                          */
#define TFT_DC_GPIO   GPIO_NUM_14   /* D_DC:   数据/命令选择                     */
#define TFT_BLK_GPIO  GPIO_NUM_47   /* D_BLK:  背光 PWM 正相(高=亮) ~1kHz       */

// FAN1 (4-pin PWM, Intel 规范, 25kHz 开漏反相)
#define FAN1_PWM_GPIO  GPIO_NUM_38  /* PWM  (反相: MCU高→NMOS导通→风扇0%)       */
#define FAN1_TACH_GPIO GPIO_NUM_40  /* TACH (PCNT 计数, 10k↑3V3)               */

// FAN2
#define FAN2_PWM_GPIO  GPIO_NUM_39  /* PWM  (反相)                              */
#define FAN2_TACH_GPIO GPIO_NUM_41  /* TACH                                     */