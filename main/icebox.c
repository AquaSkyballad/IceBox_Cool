/**
 * @file    icebox.c
 * @brief   TEC半导体冰箱控制系统主程序
 * 
 * ESP32-S3N16R8 + ESP-IDF 5.5.5
 * 硬件测试和业务代码通过menuconfig选择编译
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include "gpio_def.h"




static const char *TAG = "icebox";

/* 全局事件队列 */


void app_main(void)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
