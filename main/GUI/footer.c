/*
 * footer.c
 *
 *  Created on: 29 сент. 2025?г.
 *      Author: farid
 */


#include "footer.h"
#include <stdio.h>
#include "lvgl.h"
#include "settings.h"
#include "mainScreen.h"

LV_IMG_DECLARE(LefArrow);
LV_IMG_DECLARE(Circle);
LV_IMG_DECLARE(Menu);

static void btnHomeEventCB(lv_event_t *e) {
    createMainScreen();
    printf("go to main screen");
    fflush(stdout);
}

void createFooter() {
    lv_obj_t *footer = lv_obj_create(lv_screen_active());
    lv_obj_set_size(footer,LCD_H_RES ,FOOTER_HEIGHT);
    lv_obj_align(footer, LV_ALIGN_BOTTOM_LEFT, 0, 0);
    lv_obj_set_style_bg_color(footer, FOOTER_COLOR, LV_PART_MAIN);
    lv_obj_set_style_border_width(footer, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(footer, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(footer, 0, LV_PART_MAIN);
    lv_obj_clear_flag(footer, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *label = lv_label_create(footer);
    lv_label_set_text(label, "Footer!");
    lv_obj_align(label,LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_text_color(label, lv_color_white(),LV_PART_MAIN);

    //lv_obj_set_size(butHome, LCD_H_RES / 3 - 5, FOOTER_HEIGHT - 2);
    lv_obj_t *butBack = lv_button_create(footer);
    lv_obj_set_size(butBack, LCD_H_RES / 3 - 5, FOOTER_HEIGHT - 2);
    lv_obj_align(butBack, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_t *imgLefArrow = lv_img_create(butBack);
    lv_img_set_src(imgLefArrow, &LefArrow);
    lv_obj_align(imgLefArrow, LV_ALIGN_CENTER, 0, 0);

    lv_obj_t *butHome = lv_button_create(footer);
    lv_obj_set_size(butHome, LCD_H_RES / 3 - 5, FOOTER_HEIGHT - 2);
    lv_obj_align(butHome, LV_ALIGN_CENTER, 0, 0);
    lv_obj_t *imgCircle = lv_img_create(butHome);
    lv_img_set_src(imgCircle, &Circle);
    lv_obj_align(imgCircle, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_event_cb(butHome, btnHomeEventCB, LV_EVENT_CLICKED, NULL);

    lv_obj_t *butMenu = lv_button_create(footer);
    lv_obj_set_size(butMenu, LCD_H_RES / 3 - 5, FOOTER_HEIGHT - 2);
    lv_obj_align(butMenu, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_t *imgMenu = lv_img_create(butMenu);
    lv_img_set_src(imgMenu, &Menu);
    lv_obj_align(imgMenu, LV_ALIGN_CENTER, 0, 0);
}
