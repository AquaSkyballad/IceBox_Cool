/**
 * @file    NCP81239.c
 * @brief   NCP81239 4-Switch Buck-Boost Controller 驱动实现
 */

#include "NCP81239.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"

static const char *TAG = "ncp81239";

/* ================================================================
 *  内部辅助 — I²C 读写
 * ================================================================ */

/**
 * @brief 写单个寄存器
 */
static esp_err_t reg_write(ncp81239_t *dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(dev->i2c_handle, buf, sizeof(buf), -1);
}

/**
 * @brief 读单个寄存器
 */
static esp_err_t reg_read(ncp81239_t *dev, uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(dev->i2c_handle, &reg, 1, val, 1, -1);
}

/**
 * @brief 读-改-写寄存器中指定 mask 的位
 */
static esp_err_t reg_update(ncp81239_t *dev, uint8_t reg, uint8_t mask, uint8_t val)
{
    uint8_t cur;
    ESP_RETURN_ON_ERROR(reg_read(dev, reg, &cur), TAG, "reg_read 0x%02X", reg);
    uint8_t updated = (cur & ~mask) | (val & mask);
    if (updated == cur) return ESP_OK;           /* 无变化则跳过 */
    return reg_write(dev, reg, updated);
}

/* ================================================================
 *  公开 API
 * ================================================================ */

esp_err_t ncp81239_init(ncp81239_t *dev, i2c_port_t i2c_port,
                        gpio_num_t scl_gpio, gpio_num_t sda_gpio,
                        gpio_num_t en_gpio, gpio_num_t int_gpio)
{
    ESP_RETURN_ON_FALSE(dev != NULL, ESP_ERR_INVALID_ARG, TAG, "dev is NULL");

    memset(dev, 0, sizeof(*dev));
    dev->i2c_port  = i2c_port;
    dev->i2c_addr  = NCP81239_I2C_ADDR;
    dev->en_gpio   = en_gpio;
    dev->int_gpio  = int_gpio;

    /* ---- 1. GPIO: P_EN=低 (禁用功率级), INT=输入 ---- */
    gpio_config_t io_cfg = {
        .pin_bit_mask = BIT64(en_gpio),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_cfg), TAG, "EN GPIO config");
    gpio_set_level(en_gpio, 0);                      /* 默认关断 */

    io_cfg.pin_bit_mask = BIT64(int_gpio);
    io_cfg.mode          = GPIO_MODE_INPUT;
    io_cfg.pull_up_en    = GPIO_PULLUP_DISABLE;       /* 外部 10k 上拉 */
    io_cfg.intr_type     = GPIO_INTR_NEGEDGE;         /* 下降沿中断 */
    ESP_RETURN_ON_ERROR(gpio_config(&io_cfg), TAG, "INT GPIO config");

    /* ---- 2. I²C 总线 ---- */
    i2c_master_bus_config_t bus_cfg = {
        .clk_source     = I2C_CLK_SRC_DEFAULT,
        .i2c_port       = i2c_port,
        .scl_io_num     = scl_gpio,
        .sda_io_num     = sda_gpio,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,         /* 外部 4.7k 上拉 */
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &dev->bus_handle), TAG, "I2C bus");

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = NCP81239_I2C_ADDR,
        .scl_speed_hz    = 400000,                      /* 400kHz Fast-mode */
    };
    ESP_RETURN_ON_ERROR(
        i2c_master_bus_add_device(dev->bus_handle, &dev_cfg, &dev->i2c_handle),
        TAG, "I2C add device");

    /* ---- 3. 寄存器初始化序列 (circuit.md §7) ---- */
    /* 3a. 05h: CLIP=11 (70mV/35A) + CLIN=00 (-40mV/-20A)  ★必须 */
    ESP_RETURN_ON_ERROR(
        reg_write(dev, NCP81239_REG_ILIMIT,
                  (NCP81239_CLIN_N40MV << 4) | NCP81239_CLIP_70MV),
        TAG, "write ILIMIT");

    /* 3b. 03h: 600kHz + dac_lsb=0 (10mV 步进) */
    ESP_RETURN_ON_ERROR(
        reg_write(dev, NCP81239_REG_FREQ_DAC_LSB, NCP81239_FREQ_600KHZ),
        TAG, "write FREQ");

    /* 3c. 02h: 压摆率默认 0.6mV/µs */
    ESP_RETURN_ON_ERROR(
        reg_write(dev, NCP81239_REG_SLEW_RATE, NCP81239_SLEW_0_6MV_US),
        TAG, "write SLEW");

    /* 3d. 01h: 初始输出电压 5V (dac_code=0x40, V_DAC≈637mV) */
    ESP_RETURN_ON_ERROR(
        reg_write(dev, NCP81239_REG_DAC_TARGET, 0x40),
        TAG, "write DAC");

    /* 3e. 09h: 屏蔽 CLIND (本板悬空) + 其它中断先不屏蔽 */
    ESP_RETURN_ON_ERROR(
        reg_write(dev, NCP81239_REG_INT_MASK, NCP81239_INT_CLIND),
        TAG, "write INT_MASK");

    /* 3f. 0Ah: 屏蔽 shutdown 中断 (上电阶段) */
    ESP_RETURN_ON_ERROR(
        reg_write(dev, NCP81239_REG_INT_MASK2, NCP81239_INT_SHUTDOWN),
        TAG, "write INT_MASK2");

    /* 3g. 04h: 关闭死电池/CFET/PFET */
    ESP_RETURN_ON_ERROR(
        reg_write(dev, NCP81239_REG_FUNC_ENABLE, 0x00),
        TAG, "write FUNC_ENABLE");

    /* 3h. 08h: ADC 默认选 VFB + 连续读 (使能前配置) */
    ESP_RETURN_ON_ERROR(
        reg_write(dev, NCP81239_REG_ADC_CTRL,
                  NCP81239_AMUX_VFB | NCP81239_ADC_TRIG_CONT),
        TAG, "write ADC_CTRL");

    dev->initialized = true;
    ESP_LOGI(TAG, "init OK (addr=0x%02X, scl=%d, sda=%d, en=%d, int=%d)",
             NCP81239_I2C_ADDR, scl_gpio, sda_gpio, en_gpio, int_gpio);
    return ESP_OK;
}

esp_err_t ncp81239_deinit(ncp81239_t *dev)
{
    ESP_RETURN_ON_FALSE(dev && dev->initialized, ESP_ERR_INVALID_STATE, TAG, "not initialized");

    /* 先关断功率级 */
    ncp81239_disable(dev);

    if (dev->i2c_handle) {
        i2c_master_bus_rm_device(dev->i2c_handle);
        dev->i2c_handle = NULL;
    }
    if (dev->bus_handle) {
        i2c_del_master_bus(dev->bus_handle);
        dev->bus_handle = NULL;
    }

    gpio_reset_pin(dev->en_gpio);
    gpio_reset_pin(dev->int_gpio);

    dev->initialized = false;
    return ESP_OK;
}

/* ---- 使能 / 禁用 ---- */

esp_err_t ncp81239_enable(ncp81239_t *dev)
{
    ESP_RETURN_ON_FALSE(dev && dev->initialized, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    gpio_set_level(dev->en_gpio, 1);
    dev->enabled = true;

    /* ★ EN 拉高后延时，待内部模拟稳定再解除 CLIND 屏蔽 (见坑#5) */
    vTaskDelay(pdMS_TO_TICKS(5));
    reg_update(dev, NCP81239_REG_INT_MASK, NCP81239_INT_CLIND, 0x00);

    ESP_LOGI(TAG, "power stage enabled");
    return ESP_OK;
}

esp_err_t ncp81239_disable(ncp81239_t *dev)
{
    ESP_RETURN_ON_FALSE(dev && dev->initialized, ESP_ERR_INVALID_STATE, TAG, "not initialized");

    /* 拉低前先屏蔽 CLIND，防止抖动 */
    reg_update(dev, NCP81239_REG_INT_MASK, NCP81239_INT_CLIND, NCP81239_INT_CLIND);

    gpio_set_level(dev->en_gpio, 0);
    dev->enabled = false;

    ESP_LOGI(TAG, "power stage disabled");
    return ESP_OK;
}

/* ---- 电压设定 ---- */

esp_err_t ncp81239_set_voltage(ncp81239_t *dev, float volt, bool use_5mv_lsb)
{
    ESP_RETURN_ON_FALSE(dev && dev->initialized, ESP_ERR_INVALID_STATE, TAG, "not initialized");

    uint8_t dac_code;
    bool    lsb;
    if (!ncp81239_voltage_to_dac(volt, use_5mv_lsb, &dac_code, &lsb)) {
        ESP_LOGE(TAG, "voltage %.3fV out of range (0.78~20.01V)", volt);
        return ESP_ERR_INVALID_ARG;
    }

    /* 先写 LSB 位 (03h.bit4) */
    uint8_t val = (lsb ? NCP81239_DAC_TARGET_LSB : 0x00) | NCP81239_FREQ_600KHZ;
    ESP_RETURN_ON_ERROR(
        reg_write(dev, NCP81239_REG_FREQ_DAC_LSB, val), TAG, "write freq+lsb");

    /* 再写 DAC 码 (01h) */
    ESP_RETURN_ON_ERROR(
        reg_write(dev, NCP81239_REG_DAC_TARGET, dac_code), TAG, "write dac");

    float actual = ncp81239_dac_to_voltage(dac_code, lsb);
    ESP_LOGI(TAG, "set Vout=%.2fV (dac=0x%02X, lsb=%d)", actual, dac_code, lsb);
    return ESP_OK;
}

esp_err_t ncp81239_get_dac_code(ncp81239_t *dev, uint8_t *code, bool *lsb)
{
    ESP_RETURN_ON_FALSE(dev && dev->initialized, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    ESP_RETURN_ON_FALSE(code && lsb, ESP_ERR_INVALID_ARG, TAG, "NULL output");

    uint8_t dac, freq_lsb;
    ESP_RETURN_ON_ERROR(reg_read(dev, NCP81239_REG_DAC_TARGET, &dac), TAG, "read dac");
    ESP_RETURN_ON_ERROR(reg_read(dev, NCP81239_REG_FREQ_DAC_LSB, &freq_lsb), TAG, "read freq");

    *code = dac;
    *lsb  = (freq_lsb & NCP81239_DAC_TARGET_LSB) != 0;
    return ESP_OK;
}

/* ---- 电流限 ---- */

esp_err_t ncp81239_set_ilimit(ncp81239_t *dev, uint8_t clip, uint8_t clin)
{
    ESP_RETURN_ON_FALSE(dev && dev->initialized, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    uint8_t val = ((clin & 0x03) << 4) | (clip & 0x03);
    ESP_RETURN_ON_ERROR(
        reg_write(dev, NCP81239_REG_ILIMIT, val), TAG, "write ILIMIT");
    ESP_LOGI(TAG, "set ilimit: CLIP=%d, CLIN=%d", clip, clin);
    return ESP_OK;
}

/* ---- CLIND 外部比较器 ---- */

esp_err_t ncp81239_set_clind_threshold(ncp81239_t *dev,
                                       uint8_t cs1_threshold,
                                       uint8_t cs2_threshold)
{
    ESP_RETURN_ON_FALSE(dev && dev->initialized, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    uint8_t val = ((cs2_threshold & 0x03) << 2) | (cs1_threshold & 0x03);
    ESP_RETURN_ON_ERROR(
        reg_write(dev, NCP81239_REG_CLIND, val), TAG, "write CLIND");
    return ESP_OK;
}

/* ---- 压摆率 ---- */

esp_err_t ncp81239_set_slew_rate(ncp81239_t *dev, uint8_t slew_rate)
{
    ESP_RETURN_ON_FALSE(dev && dev->initialized, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    ESP_RETURN_ON_ERROR(
        reg_write(dev, NCP81239_REG_SLEW_RATE, slew_rate & 0x03), TAG, "write SLEW");
    ESP_LOGI(TAG, "set slew_rate=%d", slew_rate & 0x03);
    return ESP_OK;
}

/* ---- 频率 ---- */

esp_err_t ncp81239_set_frequency(ncp81239_t *dev, uint8_t freq)
{
    ESP_RETURN_ON_FALSE(dev && dev->initialized, ESP_ERR_INVALID_STATE, TAG, "not initialized");

    /* 读当前 dac_lsb ，保留之 */
    uint8_t cur;
    ESP_RETURN_ON_ERROR(
        reg_read(dev, NCP81239_REG_FREQ_DAC_LSB, &cur), TAG, "read freq");
    uint8_t val = (cur & NCP81239_DAC_TARGET_LSB) | (freq & 0x07);
    ESP_RETURN_ON_ERROR(
        reg_write(dev, NCP81239_REG_FREQ_DAC_LSB, val), TAG, "write freq");
    ESP_LOGI(TAG, "set frequency=%d", freq & 0x07);
    return ESP_OK;
}

/* ---- gm 补偿 ---- */

esp_err_t ncp81239_set_gm(ncp81239_t *dev, uint8_t gm_value, bool manual)
{
    ESP_RETURN_ON_FALSE(dev && dev->initialized, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    uint8_t val = NCP81239_GM_AMP_CONFIG              /* bit7: amp config */
                | ((gm_value & 0x07) << 4)             /* [6:4] hi_gm   */
                | (manual ? BIT(3) : 0x00)             /* [3] manual    */
                | (gm_value & 0x07);                   /* [2:0] lo_gm   */
    ESP_RETURN_ON_ERROR(
        reg_write(dev, NCP81239_REG_GM_COMP, val), TAG, "write GM");
    ESP_LOGI(TAG, "set gm=%d (manual=%d)", gm_value & 0x07, manual);
    return ESP_OK;
}

/* ---- ADC ---- */

esp_err_t ncp81239_config_adc(ncp81239_t *dev, uint8_t amux, uint8_t trigger)
{
    ESP_RETURN_ON_FALSE(dev && dev->initialized, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    uint8_t val = (amux & 0x1C) | (trigger & 0x03);
    ESP_RETURN_ON_ERROR(
        reg_write(dev, NCP81239_REG_ADC_CTRL, val), TAG, "write ADC_CTRL");
    return ESP_OK;
}

esp_err_t ncp81239_read_telemetry(ncp81239_t *dev,
                                  float *volt, float *vin,
                                  float *iout, float *iin)
{
    ESP_RETURN_ON_FALSE(dev && dev->initialized, ESP_ERR_INVALID_STATE, TAG, "not initialized");

    uint8_t raw;
    float   v, inp, io, ii;

    /* VFB → 10h */
    ESP_RETURN_ON_ERROR(
        ncp81239_config_adc(dev, NCP81239_AMUX_VFB, NCP81239_ADC_TRIG_ONCE),
        TAG, "adc VFB");
    vTaskDelay(pdMS_TO_TICKS(1));
    ESP_RETURN_ON_ERROR(reg_read(dev, NCP81239_REG_VFB, &raw), TAG, "read VFB");
    v = ncp81239_raw_to_vout(raw);

    /* VIN → 11h */
    ESP_RETURN_ON_ERROR(
        ncp81239_config_adc(dev, NCP81239_AMUX_VIN, NCP81239_ADC_TRIG_ONCE),
        TAG, "adc VIN");
    vTaskDelay(pdMS_TO_TICKS(1));
    ESP_RETURN_ON_ERROR(reg_read(dev, NCP81239_REG_VIN, &raw), TAG, "read VIN");
    inp = ncp81239_raw_to_vin(raw);

    /* CS2 (Iout) → 12h */
    ESP_RETURN_ON_ERROR(
        ncp81239_config_adc(dev, NCP81239_AMUX_CS2, NCP81239_ADC_TRIG_ONCE),
        TAG, "adc CS2");
    vTaskDelay(pdMS_TO_TICKS(1));
    ESP_RETURN_ON_ERROR(reg_read(dev, NCP81239_REG_CS2, &raw), TAG, "read CS2");
    io = ncp81239_raw_to_current(raw);

    /* CS1 (Iin) → 13h */
    ESP_RETURN_ON_ERROR(
        ncp81239_config_adc(dev, NCP81239_AMUX_CS1, NCP81239_ADC_TRIG_ONCE),
        TAG, "adc CS1");
    vTaskDelay(pdMS_TO_TICKS(1));
    ESP_RETURN_ON_ERROR(reg_read(dev, NCP81239_REG_CS1, &raw), TAG, "read CS1");
    ii = ncp81239_raw_to_current(raw);

    /* 恢复默认: VFB + 连续读 */
    ncp81239_config_adc(dev, NCP81239_AMUX_VFB, NCP81239_ADC_TRIG_CONT);

    if (volt) *volt = v;
    if (vin)  *vin  = inp;
    if (iout) *iout = io;
    if (iin)  *iin  = ii;

    return ESP_OK;
}

/* ---- 中断 ---- */

esp_err_t ncp81239_set_int_mask(ncp81239_t *dev, uint8_t mask, uint8_t mask2)
{
    ESP_RETURN_ON_FALSE(dev && dev->initialized, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    ESP_RETURN_ON_ERROR(
        reg_write(dev, NCP81239_REG_INT_MASK, mask), TAG, "write INT_MASK");
    ESP_RETURN_ON_ERROR(
        reg_write(dev, NCP81239_REG_INT_MASK2, mask2), TAG, "write INT_MASK2");
    return ESP_OK;
}

esp_err_t ncp81239_read_int_status(ncp81239_t *dev, ncp81239_int_status_t *status)
{
    ESP_RETURN_ON_FALSE(dev && dev->initialized, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    ESP_RETURN_ON_FALSE(status, ESP_ERR_INVALID_ARG, TAG, "NULL status");

    uint8_t stat14, stat15;

    /*
     * 手册要求: 先把 mask 写 1（屏蔽），此时中断源寄存器才会锁存故障。
     * 本实现使用 09h/0Ah 的当前屏蔽值，写全 1 再恢复。
     */
    uint8_t saved_mask, saved_mask2;
    reg_read(dev, NCP81239_REG_INT_MASK, &saved_mask);
    reg_read(dev, NCP81239_REG_INT_MASK2, &saved_mask2);

    /* 全屏蔽，锁存中断源 */
    reg_write(dev, NCP81239_REG_INT_MASK, 0xFF);
    reg_write(dev, NCP81239_REG_INT_MASK2, 0xFF);

    /* 读取状态 */
    ESP_RETURN_ON_ERROR(reg_read(dev, NCP81239_REG_STATUS, &stat14), TAG, "read STATUS");
    ESP_RETURN_ON_ERROR(reg_read(dev, NCP81239_REG_SHUTDOWN, &stat15), TAG, "read SHUTDOWN");

    /* 恢复屏蔽 */
    reg_write(dev, NCP81239_REG_INT_MASK, saved_mask);
    reg_write(dev, NCP81239_REG_INT_MASK2, saved_mask2);

    memset(status, 0, sizeof(*status));
    if (stat14 & NCP81239_STAT_I2C_ACK) status->i2c_ack = true;
    if (stat14 & NCP81239_STAT_VCHN)    status->vchn    = true;
    if (stat14 & NCP81239_STAT_TSD)     status->tsd     = true;
    if (stat14 & NCP81239_STAT_PG)      status->pg      = true;
    if (stat14 & NCP81239_STAT_OCP_P)   status->ocp_p   = true;
    if (stat14 & NCP81239_STAT_OV)      status->ov      = true;
    if (stat14 & NCP81239_STAT_CLIND)   status->clind   = true;
    if (stat15 & NCP81239_SHUTDOWN_BIT) status->shutdown = true;

    return ESP_OK;
}

esp_err_t ncp81239_clear_interrupts(ncp81239_t *dev)
{
    ESP_RETURN_ON_FALSE(dev && dev->initialized, ESP_ERR_INVALID_STATE, TAG, "not initialized");

    /* 全屏蔽再解除，等效清中断 */
    ESP_RETURN_ON_ERROR(
        reg_write(dev, NCP81239_REG_INT_MASK, 0xFF), TAG, "clear mask");
    ESP_RETURN_ON_ERROR(
        reg_write(dev, NCP81239_REG_INT_MASK2, 0xFF), TAG, "clear mask2");
    vTaskDelay(pdMS_TO_TICKS(1));

    /* 恢复默认: 仅屏蔽 CLIND */
    ESP_RETURN_ON_ERROR(
        reg_write(dev, NCP81239_REG_INT_MASK, NCP81239_INT_CLIND), TAG, "restore mask");
    ESP_RETURN_ON_ERROR(
        reg_write(dev, NCP81239_REG_INT_MASK2, 0x00), TAG, "restore mask2");

    return ESP_OK;
}