/*
 * lvglTask.c
 *
 *  Created on: 18 сент. 2025?г.
 *      Author: farid
 */
#include "lvglTask.h"
#include "display/lv_display.h"
#include "driver/gpio.h"
#include "esp_lcd_io_i80.h"
#include "esp_lcd_panel_dev.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_st7796.h"
#include "esp_lcd_types.h"
#include "esp_log.h"
#include "freertos/projdefs.h"
#include "lvgl.h"
#include "lv_init.h"
#include "misc/lv_color.h"
#include "soc/gpio_num.h"
#include "freertos/FreeRTOS.h"
#include "widgets/label/lv_label.h"


#define TAG "LVGL_TASK"

static lv_color_t buf1[LCD_H_RES * 40];
static lv_color_t buf2[LCD_H_RES * 40];
static esp_lcd_i80_bus_handle_t i80_bus = NULL;
static esp_lcd_panel_io_handle_t io_handle = NULL;
static esp_lcd_panel_handle_t panel_handle = NULL;

static void my_flush_cb(lv_display_t *disp, const lv_area_t *area,
						uint8_t *px_map) {
	esp_lcd_panel_handle_t panel =
		(esp_lcd_panel_handle_t)lv_display_get_user_data(disp);

	// Размер области
	int width = area->x2 - area->x1 + 1;
	int height = area->y2 - area->y1 + 1;
	//size_t len = width * height * 2; // RGB565 = 2 байта на пиксель

//	// Перестановка байтов вручную
//	for (size_t i = 0; i < len; i += 2) {
//		uint8_t tmp = px_map[i];
//		px_map[i] = px_map[i + 1];
//		px_map[i + 1] = tmp;
//	}

	// Отправка буфера в дисплей
	esp_lcd_panel_draw_bitmap(panel, area->x1, area->y1, area->x2 + 1,
							  area->y2 + 1, px_map);

	// Сообщаем LVGL, что отрисовка завершена
	lv_display_flush_ready(disp);
}

static void create_i80_bus() {
	// 1. Create I80 bus
	ESP_LOGI(TAG, "1. Create I80 bus");

	esp_lcd_i80_bus_config_t bus_config = {
		.clk_src = LCD_CLK_SRC_PLL160M, // LCD_CLK_SRC_DEFAULT,
		.dc_gpio_num = GPIO_NUM_0,
		.wr_gpio_num = GPIO_NUM_47,
		.data_gpio_nums = {GPIO_NUM_9, GPIO_NUM_46, GPIO_NUM_3, GPIO_NUM_8,
						   GPIO_NUM_18, GPIO_NUM_17, GPIO_NUM_16, GPIO_NUM_15},
		.bus_width = 8,
		.max_transfer_bytes = LCD_H_RES * LCD_V_RES * sizeof(uint16_t),
		//.dma_burst_size = 256, // 128 64
	};
	ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bus_config, &i80_bus));
	// vTaskDelay(pdMS_TO_TICKS(1000));
}

static void createLCDIODevice() {
	// 2. Allocate an LCD IO device handle from the I80 bus
	ESP_LOGI(TAG, "2. Allocate an LCD IO device handle from the I80 bus");

	esp_lcd_panel_io_i80_config_t io_config = {
		.cs_gpio_num = GPIO_NUM_NC,
		.pclk_hz = 10 * 1000 * 1000, // pclk_hz = (H_RES × V_RES × BPP × FPS),
									 // BPP- Byte per pixel (2)
		.trans_queue_depth = 10, // trans_queue_depth ≈ (макс. строк в буфере /
								 // строк на одну DMA-передачу)
		.dc_levels =
			{
				.dc_cmd_level = 0,
				.dc_data_level = 1,
				.dc_dummy_level = 0,
				.dc_idle_level = 0,
			},
		.lcd_cmd_bits = 8,
		.lcd_param_bits = 8, 
		.flags.swap_color_bytes = true,
		.flags.reverse_color_bits = false ,
	};
	ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus, &io_config, &io_handle));
}

static void installLCDControllerDriver() {
	// 3. Install the LCD controller driver
	ESP_LOGI(TAG, "3. Install the LCD controller driver");
	
	esp_lcd_panel_dev_config_t panel_config = {
		.reset_gpio_num = GPIO_NUM_4,
		.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
		.bits_per_pixel = 16,
	};
	ESP_ERROR_CHECK(
		esp_lcd_new_panel_st7796(io_handle, &panel_config, &panel_handle));
	esp_lcd_panel_reset(panel_handle);
	esp_lcd_panel_init(panel_handle);
	esp_lcd_panel_mirror(panel_handle, false, true);
	esp_lcd_panel_invert_color(panel_handle, true);
	ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
}

static void switchBacklightOn() {
	// 4. Switch backlight ON
	gpio_set_direction(GPIO_NUM_45, GPIO_MODE_OUTPUT); // backlight
	gpio_set_level(GPIO_NUM_45, 1);
}

static void initLVGL() {
    // 5. LVGL init
  /*
  lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
  ESP_ERROR_CHECK(lvgl_port_init(&lvgl_cfg));
  const lvgl_port_display_cfg_t disp_cfg = {
    .io_handle = io_handle,
    .panel_handle = panel_handle,
    .buffer_size = LCD_H_RES * 40,
    .double_buffer = true,
    .hres = LCD_H_RES,
    .vres = LCD_V_RES,
    .color_format = LV_COLOR_FORMAT_RGB565,
    .rotation.mirror_x = true,
    .rotation.swap_xy = false,
    .rotation.mirror_y = false,

    .flags.buff_dma = true,
    .flags.swap_bytes = true,
    .flags.sw_rotate = false,
  };
  lv_disp_t *disp = lvgl_port_add_disp(&disp_cfg);
  */
  
	lv_init();
	lv_display_t *disp = lv_display_create(LCD_H_RES, LCD_V_RES);
	lv_display_set_flush_cb(disp, my_flush_cb);
	lv_display_set_buffers(disp, buf1, buf2, sizeof(buf1),
						   LV_DISPLAY_RENDER_MODE_PARTIAL);
	lv_display_set_user_data(disp, panel_handle); // передаём дисплейный хэндл
}

void initDispaly() {
  create_i80_bus();
  createLCDIODevice();
  installLCDControllerDriver();
  switchBacklightOn();
  initLVGL();
}

void lvgl_task(void *pvParameters) {
    initDispaly();
    
      // 6. Create interface
  lv_obj_t *label = lv_label_create(lv_scr_act());
  lv_label_set_text(label, "Hello World");
  lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

  // Контейнер для вертикального размещения
  lv_obj_t *container = lv_obj_create(lv_scr_act());
  lv_obj_set_size(container, LCD_H_RES, LCD_V_RES);
  lv_obj_set_flex_flow(container,
             LV_FLEX_FLOW_COLUMN); // вертикальное размещение
  lv_obj_set_flex_align(container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
              LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_row(container, 20, 0); // отступ между кнопками

  // 🔴 Красная кнопка
  lv_obj_t *btn_red = lv_btn_create(container);
  lv_obj_set_size(btn_red, 200, 60);
  lv_obj_set_style_bg_color(btn_red, lv_color_make(255, 0, 0), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(btn_red, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_t *label_red = lv_label_create(btn_red);
  lv_label_set_text(label_red, "Red");

  // 🟢 Зелёная кнопка
  lv_obj_t *btn_green = lv_btn_create(container);
  lv_obj_set_size(btn_green, 200, 60);
  lv_obj_set_style_bg_color(btn_green, lv_color_make(0, 255, 0),
                LV_PART_MAIN);
  lv_obj_set_style_bg_opa(btn_green, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_t *label_green = lv_label_create(btn_green);
  lv_label_set_text(label_green, "Green");

  // 🔵 Синяя кнопка
  lv_obj_t *btn_blue = lv_btn_create(container);
  lv_obj_set_size(btn_blue, 200, 60);
  lv_obj_set_style_bg_color(btn_blue, lv_color_make(0, 0, 255), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(btn_blue, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_t *label_blue = lv_label_create(btn_blue);
  lv_label_set_text(label_blue, "Blue");

    while (true) {
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

