/*
 * header.c
 *
 *  Created on: 21 сент. 2025?г.
 *      Author: farid
 */

#include "header.h"
#include "stdio.h"
#include "lvgl.h"
#include "settings.h"
#include "signalStrengh.h"

static lv_obj_t *label;

void createHeader() {
	lv_obj_t *topPanelBar_obj = lv_obj_create(lv_screen_active());
	lv_obj_align(topPanelBar_obj, LV_ALIGN_TOP_LEFT, 0, 0);
	lv_obj_set_size(topPanelBar_obj, LCD_H_RES, HEADER_HEIGHT);
	lv_obj_set_style_bg_color(topPanelBar_obj, HEADER_COLOR, LV_PART_MAIN);
	lv_obj_set_style_bg_opa(topPanelBar_obj, LV_OPA_COVER, LV_PART_MAIN);
	lv_obj_set_style_border_width(topPanelBar_obj, 0, LV_PART_MAIN);
	lv_obj_set_style_pad_all(topPanelBar_obj, 0, LV_PART_MAIN);
	lv_obj_set_style_radius(topPanelBar_obj, 0, LV_PART_MAIN);
	//lv_obj_set_layout(topPanelBar_obj, LV_LAYOUT_NONE); // отключаем layout

	lv_obj_t *signal = create_signal_indicator(topPanelBar_obj, 5, 30, HEADER_HEIGHT - 4);
	lv_obj_align(signal, LV_ALIGN_RIGHT_MID, -2, 0);

	label = lv_label_create(topPanelBar_obj);
	lv_label_set_text(label, "Main screen");
	lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
	lv_obj_set_style_text_color(label, lv_color_white(), LV_PART_MAIN );
}

void headerSetText(const char *txt) {
    if (label) {
        lv_label_set_text(label, txt);
    }else{
        printf("ERROR: there is no label object");
    }
}
