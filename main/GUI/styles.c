/*
 * styles.c
 *
 *  Created on: 30 сент. 2025?г.
 *      Author: farid
 */

#include "styles.h"
#include "lvgl.h"



static lv_style_t styleBtnIcon;

static void initIconButtonStyle() {
    lv_style_init(&styleBtnIcon);

    lv_style_set_radius(&styleBtnIcon, 6); // скругление
    lv_style_set_bg_color(&styleBtnIcon, lv_color_hex(0x777777)); // фон
    lv_style_set_bg_opa(&styleBtnIcon, LV_OPA_50);

    lv_style_set_border_width(&styleBtnIcon, 2);
    lv_style_set_border_color(&styleBtnIcon, lv_color_white());

    lv_style_set_text_color(&styleBtnIcon, lv_color_white());
    lv_style_set_text_font(&styleBtnIcon, &lv_font_montserrat_14); // или свой шрифт

    lv_style_set_size(&styleBtnIcon, BTN_ICON_SIZE, BTN_ICON_SIZE);
    lv_style_set_pad_all(&styleBtnIcon,0);

    lv_style_set_layout(&styleBtnIcon, LV_LAYOUT_FLEX);
    lv_style_set_flex_flow(&styleBtnIcon, LV_FLEX_FLOW_COLUMN);

}

lv_style_t *getStyleBtnIcon() {
    return &styleBtnIcon;
}

void initSyles() {
    initIconButtonStyle();
}

