/*
 * mainPanel.c
 *
 *  Created on: 30 сент. 2025?г.
 *      Author: farid
 */
#include "mainPanel.h"
#include "lvgl.h"
#include "settings.h"

static lv_obj_t *mainPanel;

lv_obj_t *createMainPanel() {
    mainPanel = lv_obj_create(lv_screen_active());
    lv_obj_set_size(mainPanel, LCD_H_RES, MAIN_PANEL_HEIGHT);
    lv_obj_align(mainPanel, LV_ALIGN_TOP_LEFT, 0, HEADER_HEIGHT);
    lv_obj_set_style_bg_color(mainPanel, MAIN_PANEL_BG_COLOR, LV_PART_MAIN);
    lv_obj_set_style_radius(mainPanel, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(mainPanel, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(mainPanel, 0, LV_PART_MAIN);
    return mainPanel;
}

lv_obj_t *getMainPanel() {
    return mainPanel;
}

void destroyMainPanel() {
    if (mainPanel) {
        lv_obj_delete(mainPanel);
        mainPanel = NULL;
    }
}

