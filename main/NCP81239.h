/**
 * @file    NCP81239.h
 * @brief   NCP81239 4-Switch Buck-Boost Controller 驱动 (I²C)
 *
 * 适用于 icebox 数控电源 (ESP32-S3 + NCP81239MNTXG)
 * 硬件参数: DIV=7.847, RSENSE=2mΩ, R_CS=8.2kΩ
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 *  I²C 地址
 * ================================================================ */
#define NCP81239_I2C_ADDR       0x74    /* NCP81239MNTXG (非 A 版) */
#define NCP81239A_I2C_ADDR      0x75    /* NCP81239A (备选)       */

/* ================================================================
 *  寄存器地址 (Table 20)
 * ================================================================ */
#define NCP81239_REG_ENABLE         0x00  /* R/W: 控制/使能       */
#define NCP81239_REG_DAC_TARGET     0x01  /* R/W: 输出电压目标     */
#define NCP81239_REG_SLEW_RATE      0x02  /* R/W: 压摆率          */
#define NCP81239_REG_FREQ_DAC_LSB   0x03  /* R/W: 频率 + DAC_LSB  */
#define NCP81239_REG_FUNC_ENABLE    0x04  /* R/W: 功能使能        */
#define NCP81239_REG_ILIMIT         0x05  /* R/W: 电流限 CLIP/CLIN */
#define NCP81239_REG_CLIND          0x06  /* R/W: CLIND 比较器阈值 */
#define NCP81239_REG_GM_COMP        0x07  /* R/W: gm 补偿         */
#define NCP81239_REG_ADC_CTRL       0x08  /* R/W: ADC 控制        */
#define NCP81239_REG_INT_MASK       0x09  /* R/W: 中断屏蔽        */
#define NCP81239_REG_INT_MASK2      0x0A  /* R/W: 中断屏蔽2       */
#define NCP81239_REG_VFB            0x10  /* RO:  输出电压 ADC    */
#define NCP81239_REG_VIN            0x11  /* RO:  输入电压 ADC    */
#define NCP81239_REG_CS2            0x12  /* RO:  输出电流 ADC    */
#define NCP81239_REG_CS1            0x13  /* RO:  输入电流 ADC    */
#define NCP81239_REG_STATUS         0x14  /* RO:  状态/中断源     */
#define NCP81239_REG_SHUTDOWN       0x15  /* RO:  关断状态        */

/* ================================================================
 *  寄存器位域宏
 * ================================================================ */

/* ---- 00h EN/Control ---- */
#define NCP81239_EN_INT             BIT(3)   /* 使能中断输出     */
#define NCP81239_EN_MASK            BIT(2)   /* 使能 EN 引脚    */
#define NCP81239_EN_PUP             BIT(1)   /* EN 上拉         */
#define NCP81239_EN_POL             BIT(0)   /* EN 极性翻转     */

/* ---- 03h Frequency + DAC LSB ---- */
#define NCP81239_DAC_TARGET_LSB     BIT(4)   /* dac_target 第9位 */
#define NCP81239_FREQ_600KHZ        0x00
#define NCP81239_FREQ_150KHZ        0x01
#define NCP81239_FREQ_300KHZ        0x02
#define NCP81239_FREQ_450KHZ        0x03
#define NCP81239_FREQ_750KHZ        0x04
#define NCP81239_FREQ_900KHZ        0x05
#define NCP81239_FREQ_1200KHZ       0x06

/* ---- 04h Function Enable ---- */
#define NCP81239_CS2_DCHRG          BIT(5)
#define NCP81239_CS1_DCHRG          BIT(4)
#define NCP81239_DEAD_BATTERY_EN    BIT(2)
#define NCP81239_CFET_ON            BIT(1)
#define NCP81239_PFET_ON            BIT(0)

/* ---- 05h Current Limit ---- */
#define NCP81239_CLIP_38MV          0x00
#define NCP81239_CLIP_23MV          0x01
#define NCP81239_CLIP_11MV          0x02
#define NCP81239_CLIP_70MV          0x03  /* ★ 本板必须: 2mΩ→35A */
#define NCP81239_CLIN_N40MV         0x00
#define NCP81239_CLIN_N25MV         0x01
#define NCP81239_CLIN_N15MV         0x02
#define NCP81239_CLIN_0MV           0x03

/* ---- 09h Interrupt Mask ---- */
#define NCP81239_INT_I2C_ACK        BIT(7)
#define NCP81239_INT_VCHN           BIT(6)
#define NCP81239_INT_TSD            BIT(4)
#define NCP81239_INT_PG             BIT(3)
#define NCP81239_INT_OCP_P          BIT(2)
#define NCP81239_INT_OV             BIT(1)
#define NCP81239_INT_CLIND          BIT(0)

/* ---- 0Ah Interrupt Mask 2 ---- */
#define NCP81239_INT_SHUTDOWN       BIT(0)

/* ---- 14h Status ---- */
#define NCP81239_STAT_I2C_ACK       BIT(7)
#define NCP81239_STAT_VCHN          BIT(6)
#define NCP81239_STAT_TSD           BIT(4)
#define NCP81239_STAT_PG            BIT(3)
#define NCP81239_STAT_OCP_P         BIT(2)
#define NCP81239_STAT_OV            BIT(1)
#define NCP81239_STAT_CLIND         BIT(0)

/* ---- 15h Shutdown ---- */
#define NCP81239_SHUTDOWN_BIT       BIT(0)

/* ---- 08h ADC Control ---- */
#define NCP81239_ADC_DISABLE        BIT(5)
#define NCP81239_AMUX_VFB           0x00
#define NCP81239_AMUX_VIN           0x04
#define NCP81239_AMUX_CS2           0x08
#define NCP81239_AMUX_CS1           0x0C
#define NCP81239_ADC_TRIG_FAULT     0x00
#define NCP81239_ADC_TRIG_ONCE      0x01
#define NCP81239_ADC_TRIG_CONT      0x02

/* ---- 02h Slew Rate ---- */
#define NCP81239_SLEW_0_6MV_US      0x00
#define NCP81239_SLEW_1_2MV_US      0x01
#define NCP81239_SLEW_2_4MV_US      0x02
#define NCP81239_SLEW_4_8MV_US      0x03

/* ---- 07h gm Compensation ---- */
#define NCP81239_GM_AMP_CONFIG      BIT(7)

/* ================================================================
 *  本板硬件常数 (circuit.md + NCP81239-D datasheet)
 * ================================================================ */
#define NCP81239_FB_DIV             7.847f    /* FB 分压比 Rtop=68.47k / Rbot=10k */
#define NCP81239_RSENSE             0.002f    /* 2mΩ                         */
#define NCP81239_R_CS               8200.0f   /* 8.2kΩ                       */
#define NCP81239_GM_CS              0.005f    /* 5mS (CS 跨导)               */
#define NCP81239_ADC_LSB_MV         20.0f     /* ADC LSB=20mV                */
#define NCP81239_DAC_RES_MV         10.0f     /* DAC 默认分辨率 10mV         */
#define NCP81239_DAC_MIN_MV         100.0f    /* DAC 最低有效值 100mV (0x0A)  */
#define NCP81239_DAC_MAX_MV         2550.0f   /* DAC 最大值 2550mV (0xFF)     */

/* CS 通道增益 = RSENSE * GM_CS * R_CS */
#define NCP81239_CS_GAIN            (NCP81239_RSENSE * NCP81239_GM_CS * NCP81239_R_CS)

/* GPIO 定义 (来自 circuit.md 网表) */
#define NCP81239_P_EN_GPIO          GPIO_NUM_21    /* P_EN: 使能功率级 */
#define NCP81239_P_INT_GPIO         GPIO_NUM_42    /* INT:  中断输入  */
#define NCP81239_I2C_SCL_GPIO       GPIO_NUM_18    /* SCL              */
#define NCP81239_I2C_SDA_GPIO       GPIO_NUM_17    /* SDA              */

/* ================================================================
 *  类型定义
 * ================================================================ */

/**
 * @brief NCP81239 电流限配置 (内部路径)
 */
typedef struct {
    uint8_t clip;    /* 正电流限 CLIP (05h[1:0]) */
    uint8_t clin;    /* 负电流限 CLIN (05h[5:4]) */
} ncp81239_ilimit_t;

/**
 * @brief NCP81239 ADC 读数 (7-bit)
 */
typedef struct {
    uint8_t vfb;     /* 输出电压 ADC (10h) */
    uint8_t vin;     /* 输入电压 ADC (11h) */
    uint8_t cs2;     /* 输出电流 ADC (12h) */
    uint8_t cs1;     /* 输入电流 ADC (13h) */
} ncp81239_adc_t;

/**
 * @brief NCP81239 中断状态
 */
typedef struct {
    bool i2c_ack;
    bool vchn;
    bool tsd;
    bool pg;
    bool ocp_p;
    bool ov;
    bool clind;
    bool shutdown;
} ncp81239_int_status_t;

/**
 * @brief NCP81239 设备句柄
 */
typedef struct {
    i2c_master_dev_handle_t i2c_handle;   /* I²C 设备句柄          */
    i2c_master_bus_handle_t bus_handle;   /* I²C 总线句柄          */
    i2c_port_t              i2c_port;     /* I²C 端口号            */
    uint8_t                 i2c_addr;     /* 7-bit I²C 地址        */
    gpio_num_t              en_gpio;      /* EN 引脚               */
    gpio_num_t              int_gpio;     /* INT 引脚              */
    bool                    initialized;  /* 初始化完成标志        */
    bool                    enabled;      /* 功率级使能状态        */
} ncp81239_t;

/* ================================================================
 *  API 函数声明
 * ================================================================ */

/**
 * @brief 初始化 NCP81239 驱动实例
 *
 * I²C 总线由本函数内部初始化，调用者只需提供端口号和 GPIO。
 * 上电默认 P_EN=低，功率级禁用，需调用 ncp81239_enable() 启动。
 *
 * @param dev       设备句柄指针
 * @param i2c_port  I²C 端口号 (I2C_NUM_0 / I2C_NUM_1)
 * @param scl_gpio  SCL 引脚
 * @param sda_gpio  SDA 引脚
 * @param en_gpio   P_EN 引脚
 * @param int_gpio  INT 引脚
 * @return          ESP_OK 成功，否则失败
 */
esp_err_t ncp81239_init(ncp81239_t *dev, i2c_port_t i2c_port,
                        gpio_num_t scl_gpio, gpio_num_t sda_gpio,
                        gpio_num_t en_gpio, gpio_num_t int_gpio);

/**
 * @brief 反初始化 NCP81239 驱动，释放 I²C 资源
 */
esp_err_t ncp81239_deinit(ncp81239_t *dev);

/**
 * @brief 使能功率级 (拉高 P_EN)
 *
 * 芯片按压摆率软启动到已设置的 dac_target 电压。
 */
esp_err_t ncp81239_enable(ncp81239_t *dev);

/**
 * @brief 禁用功率级 (拉低 P_EN)
 */
esp_err_t ncp81239_disable(ncp81239_t *dev);

/**
 * @brief 设置输出电压
 *
 * @param dev        设备句柄
 * @param volt       目标输出电压 (V)，范围 0.78V ~ 20.01V
 * @param use_5mv_lsb 是否使用 5mV 精档 (false=10mV 步进)
 * @return           ESP_OK 成功，ESP_ERR_INVALID_ARG 电压越界
 *
 * @note 禁止设 volt=0 或 dac_code<0x0A。要 0V 输出请调用 ncp81239_disable()。
 */
esp_err_t ncp81239_set_voltage(ncp81239_t *dev, float volt, bool use_5mv_lsb);

/**
 * @brief 获取当前设定的 DAC 码 (从寄存器回读)
 */
esp_err_t ncp81239_get_dac_code(ncp81239_t *dev, uint8_t *code, bool *lsb);

/**
 * @brief 设置内部电流限 (CLIP / CLIN)
 *
 * @note 本板 RSENSE=2mΩ，必须设 CLIP=11 (70mV→35A)，否则 Boost 会误触发。
 */
esp_err_t ncp81239_set_ilimit(ncp81239_t *dev, uint8_t clip, uint8_t clin);

/**
 * @brief 设置外部 CLIND 比较器阈值
 *
 * @param cs1_threshold  CS1 阈值: 0=250mV, 1=750mV, 2=1.5V, 3=2.5V
 * @param cs2_threshold  CS2 阈值: 同上
 * @note 本板 CLIND 悬空，调用此函数无效，应在 09h 屏蔽 CLIND 中断。
 */
esp_err_t ncp81239_set_clind_threshold(ncp81239_t *dev,
                                       uint8_t cs1_threshold,
                                       uint8_t cs2_threshold);

/**
 * @brief 设置压摆率
 */
esp_err_t ncp81239_set_slew_rate(ncp81239_t *dev, uint8_t slew_rate);

/**
 * @brief 设置开关频率
 *
 * @param freq 频率值 (NCP81239_FREQ_xxx)
 * @note  建议先 disable → 改频 → enable，避免大电流毛刺。
 */
esp_err_t ncp81239_set_frequency(ncp81239_t *dev, uint8_t freq);

/**
 * @brief 设置 gm 补偿值
 *
 * @param gm_value  gm 值: 0=87,1=100,2=117,3=333,4=400,5=500,6=667,7=1000 μS
 * @param manual    是否手动模式 (true=手动, false=自动)
 */
esp_err_t ncp81239_set_gm(ncp81239_t *dev, uint8_t gm_value, bool manual);

/**
 * @brief 配置 ADC 通道与触发模式
 *
 * @param amux    通道选择 (NCP81239_AMUX_VFB/VIN/CS2/CS1)
 * @param trigger 触发模式 (NCP81239_ADC_TRIG_xxx)
 */
esp_err_t ncp81239_config_adc(ncp81239_t *dev, uint8_t amux, uint8_t trigger);

/**
 * @brief 读取 ADC 监测值 (物理量)
 *
 * @param dev     设备句柄
 * @param volt    输出电压 (V) 回填，可为 NULL
 * @param vin     输入电压 (V) 回填，可为 NULL
 * @param iout    输出电流 (A) 回填，可为 NULL
 * @param iin     输入电流 (A) 回填，可为 NULL
 *
 * @note 内部按 amux 顺序依次切换通道读取，耗时 ~4 次 I²C 事务。
 */
esp_err_t ncp81239_read_telemetry(ncp81239_t *dev,
                                  float *volt, float *vin,
                                  float *iout, float *iin);

/**
 * @brief 读取中断状态寄存器 (14h/15h) 并清除
 *
 * @note 读后自动清除。若 INT 引脚配置了下降沿中断，请在 ISR 中调用。
 */
esp_err_t ncp81239_read_int_status(ncp81239_t *dev, ncp81239_int_status_t *status);

/**
 * @brief 清除所有中断标志 (写 09h/0Ah 全屏蔽再解除)
 */
esp_err_t ncp81239_clear_interrupts(ncp81239_t *dev);

/**
 * @brief 配置中断屏蔽 (写入 09h + 0Ah)
 *
 * @param mask      09h 屏蔽字 (NCP81239_INT_xxx 按位或)
 * @param mask2     0Ah 屏蔽字 (NCP81239_INT_SHUTDOWN 或 0)
 * @note  bit=1 表示屏蔽该中断源
 */
esp_err_t ncp81239_set_int_mask(ncp81239_t *dev, uint8_t mask, uint8_t mask2);

/* ================================================================
 *  工具 / 换算函数 (inline / static)
 * ================================================================ */

/**
 * @brief 将物理电压转换为 DAC 码
 *
 * @param volt       目标电压 (V)
 * @param use_5mv    是否 5mV 精档
 * @param code       输出的 8-bit DAC 码
 * @param lsb        输出的 LSB 位 (仅 use_5mv=true 时有效)
 * @return           有效 DAC 码数量 (0 表示越界)
 */
static inline int ncp81239_voltage_to_dac(float volt, bool use_5mv,
                                          uint8_t *code, bool *lsb)
{
    float vdac = volt / NCP81239_FB_DIV;          /* V_DAC = Vout / 7.847 */
    float vdac_mv = vdac * 1000.0f;
    if (vdac_mv < NCP81239_DAC_MIN_MV || vdac_mv > NCP81239_DAC_MAX_MV) return 0;
    if (use_5mv) {
        int raw = (int)(vdac_mv / 5.0f + 0.5f);   /* 5mV 步进 */
        *code = (raw >> 1) & 0xFF;
        *lsb  = raw & 0x01;
    } else {
        int raw = (int)(vdac_mv / 10.0f + 0.5f);
        if (raw < 10) raw = 10;                    /* 最小 0x0A */
        *code = raw & 0xFF;
        *lsb  = false;
    }
    return 1;
}

/**
 * @brief 将 DAC 码转换为物理电压
 */
static inline float ncp81239_dac_to_voltage(uint8_t code, bool lsb)
{
    float vdac_mv = (float)code * 10.0f + (lsb ? 5.0f : 0.0f);
    return vdac_mv / 1000.0f * NCP81239_FB_DIV;
}

/**
 * @brief 将 ADC raw code (7-bit) 换算为输出电压 (V)
 */
static inline float ncp81239_raw_to_vout(uint8_t raw)
{
    return (float)(raw & 0x7F) * NCP81239_ADC_LSB_MV / 1000.0f * NCP81239_FB_DIV;
}

/**
 * @brief 将 ADC raw code (7-bit) 换算为输入电压 (V) — 内部 /10
 */
static inline float ncp81239_raw_to_vin(uint8_t raw)
{
    return (float)(raw & 0x7F) * NCP81239_ADC_LSB_MV / 1000.0f * 10.0f;
}

/**
 * @brief 将 ADC raw code (7-bit) 换算为电流 (A)
 *
 * I = V_ADC / (RSENSE * 5mS * R_CS) = raw * 20mV / 0.082
 */
static inline float ncp81239_raw_to_current(uint8_t raw)
{
    return (float)(raw & 0x7F) * NCP81239_ADC_LSB_MV / 1000.0f / NCP81239_CS_GAIN;
}

#ifdef __cplusplus
}
#endif