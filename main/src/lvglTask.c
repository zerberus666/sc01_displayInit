/*
 * lvglTask.c
 *
 *  Created on: 18 сент. 2025?г.
 *      Author: farid
 */
#include "lvglTask.h"
#include "display/lv_display.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/i2c_master.h"
#include "esp_lcd_io_i2c.h"
#include "esp_lcd_io_i80.h"
#include "esp_lcd_panel_dev.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_st7796.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_types.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "hal/i2c_types.h"
#include "indev/lv_indev.h"
#include "lv_init.h"
#include <stdbool.h>
#define LV_USE_INDEV 1
#include "lvgl.h"
#include "misc/lv_color.h"
#include "misc/lv_types.h"
#include "soc/gpio_num.h"
#include "widgets/label/lv_label.h"

#include "esp_lcd_touch_ft5x06.h"
#include <stdatomic.h>

#define TAG "LVGL_TASK"

static lv_color_t buf1[LCD_H_RES * 40];
static lv_color_t buf2[LCD_H_RES * 40];
static esp_lcd_i80_bus_handle_t i80_bus = NULL;
static esp_lcd_panel_io_handle_t io_handle = NULL;
static esp_lcd_panel_handle_t panel_handle = NULL;

static esp_lcd_touch_handle_t tp;
static lv_indev_t *indev;

static void my_flush_cb(lv_display_t *disp, const lv_area_t *area,
						uint8_t *px_map) {
	esp_lcd_panel_handle_t panel =
		(esp_lcd_panel_handle_t)lv_display_get_user_data(disp);

	// Размер области
	// int width = area->x2 - area->x1 + 1;
	// int height = area->y2 - area->y1 + 1;
	// size_t len = width * height * 2; // RGB565 = 2 байта на пиксель

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

// static void touch_read_cb(lv_indev_ *drv, lv_indev_data_t *data) {
//     uint16_t x, y;
//     bool touched = false;
//
//     esp_lcd_touch_get_coordinates(tp, &x, &y, &touched);
//
//     if (touched) {
//         data->point.x = x;
//         data->point.y = y;
//         data->state = LV_INDEV_STATE_PRESSED;
//     } else {
//         data->state = LV_INDEV_STATE_RELEASED;
//     }
// }
void my_input_read(lv_indev_t *indev, lv_indev_data_t *data) {
	ESP_LOGI(TAG, "my_input_read called");
	esp_lcd_touch_handle_t tp = lv_indev_get_user_data(indev);

	uint16_t x, y;
	uint16_t strengh;
	uint8_t pointsNum;
	bool ok =
		esp_lcd_touch_get_coordinates(tp, &x, &y, &strengh, &pointsNum, 1);
	if (ok && pointsNum > 0) {
		data->point.x = x;
		data->point.y = y;
		data->state = LV_INDEV_STATE_PRESSED;
	} else {
		data->state = LV_INDEV_STATE_RELEASED;
	}
	ESP_LOGI(TAG, "Touch: x=%d y=%d\n", x, y);
}

static void btn_red_event_cb(lv_event_t *e) {
	lv_event_code_t code = lv_event_get_code(e);
	lv_obj_t *btn = lv_event_get_target(e);

	if (code == LV_EVENT_CLICKED) {
		ESP_LOGI(TAG, "RED button pressed");
		// Можно добавить действия: смена экрана, изменение цвета и т.д.
	}
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
		.flags.reverse_color_bits = false,
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
	lv_display_set_default(disp);
}

void initTP() {
	esp_lcd_panel_io_handle_t io_handle_tp = NULL;

	//  0. i2c Master init for TP

	i2c_master_bus_handle_t i2c_bus = NULL;

	i2c_master_bus_config_t bus_config = {
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.i2c_port = I2C_NUM_0,
		.sda_io_num = GPIO_NUM_6,
		.scl_io_num = GPIO_NUM_5,
		.glitch_ignore_cnt = 7,
		.flags.enable_internal_pullup = true,
	};

	ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));

	//  1. Создаём конфигурацию I²C
	esp_lcd_panel_io_i2c_config_t io_config_tp =
		ESP_LCD_TOUCH_IO_I2C_FT5x06_CONFIG();
	io_config_tp.dev_addr = 0x38; // FT6336U адрес
	// io_config_tp.i2c_port = I2C_NUM_0; - нет!!!!
	io_config_tp.control_phase_bytes = 1;
	io_config_tp.lcd_cmd_bits = 8;
	io_config_tp.lcd_param_bits = 8;
	io_config_tp.scl_speed_hz = 400000;

	// 2. Создаём IO handle
	ESP_ERROR_CHECK(
		esp_lcd_new_panel_io_i2c(i2c_bus, &io_config_tp, &io_handle_tp));

	// 3. TP Configuration
	esp_lcd_touch_config_t tp_cfg = {
		.x_max = LCD_H_RES,
		.y_max = LCD_V_RES,
		.rst_gpio_num = -1,
		.int_gpio_num = -1,
		.levels =
			{
				.reset = 0,
				.interrupt = 0,
			},
		.flags =
			{
				.swap_xy = 0,
				.mirror_x = 0,
				.mirror_y = 0,
			},
	};

	// 4. Create touch-driver
	esp_lcd_touch_new_i2c_ft5x06(io_handle_tp, &tp_cfg, &tp);

	//----------------------
	// lv_input_device_t *touch_dev = lv_input_device_create();
	indev = lv_indev_create();
	lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
	lv_indev_set_read_cb(indev, my_input_read);
	lv_indev_set_user_data(indev, &tp);
	lv_indev_set_display(indev, lv_display_get_default());
	lv_indev_enable(indev, true);

	ESP_LOGI(TAG, "indev = %p", indev);
	assert(indev != NULL);
	ESP_LOGI(TAG, "indev active = %p", lv_indev_active());
}

void initDispaly() {
	create_i80_bus();
	createLCDIODevice();
	installLCDControllerDriver();
	switchBacklightOn();
	initLVGL();
	initTP();
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

	lv_scr_load(lv_scr_act());
	lv_obj_add_event_cb(btn_red, btn_red_event_cb, LV_EVENT_CLICKED, NULL);

	vTaskDelay(pdMS_TO_TICKS(1000));
	ESP_LOGI(TAG, "indev active = %p", lv_indev_active());

	while (true) {
		// ESP_LOGI(TAG, "lv_timer_handler running");

		lv_timer_handler();
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}
