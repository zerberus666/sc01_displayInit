/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"
#include "soc/clk_tree_defs.h"
#include "lvglTask.h"

#define TAG "---MAIN---"

void app_main(void) {
  
xTaskCreatePinnedToCore        (lvgl_task, "LVGL Task", 8192, NULL, 2, NULL, 1);

	while (true) {
		//lv_timer_handler();
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}
