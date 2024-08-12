/*
 * Copyright 2024 Emiliano Gonzalez (egonzalez . hiperion @ gmail . com))
 * * Project Site: https://github.com/hiperiondev/esp32-si5351 *
 *
 * This is based on other projects:
 *    Si5351 library for Arduino: Jason Milldrum <milldrum@gmail.com>, Dana H.
 * Myers <k6jq@comcast.net> HAL-based Si5351 driver for STM32 :
 * https://github.com/afiskon/stm32-si5351 Arduino Si5351 library tuned for size
 * and click free: https://github.com/pavelmc/Si5351mcu Others (see individual
 * files)
 *
 *    please contact their authors for more information.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "si5351.h"

static const char *TAG = "main-si5351";

si5351_t si5351_dev;

void task(void* param) {
    si5351_t* si5351_dev = (si5351_t*)param;
    si5351_i2c_init(si5351_dev, I2C_NUM_0, 18, 19, 400000);

    if (si5351_init(si5351_dev, SI5351_CRYSTAL_LOAD_6PF, SI5351_XTAL_FREQ, 0) != ESP_OK) {
        ESP_LOGI(TAG, "ERROR INIT!");
        return;
    }

    ESP_LOGI(TAG, "set freq 40MHz");
    si5351_set_freq(si5351_dev, 40000000, SI5351_CLK0);
    ESP_LOGI(TAG, "enable output");
    si5351_output_enable(si5351_dev, SI5351_CLK0, true);
    ESP_LOGI(TAG, "end init");

    while (1) {
        // printf("\n\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main() {
    // Start task
    xTaskCreate(
    task,
    "si5153_test",
    configMINIMAL_STACK_SIZE * 20,
    &si5351_dev,
    5,
    NULL
    );

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
