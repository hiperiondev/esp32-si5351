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

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <i2cdev.h>
#include <stdio.h>

#include "si5351.h"

void task(void *ignore) {
  while (1) {
    printf("\n\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void app_main() {
  // Init i2cdev library
  ESP_ERROR_CHECK(i2cdev_init());
  
  // Start task
  xTaskCreate(
    task, "i2c_scanner",
    configMINIMAL_STACK_SIZE * 3,
    NULL,
    5,
    NULL
    );
}
