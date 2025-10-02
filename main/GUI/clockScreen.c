/*
 * clockScreen.c
 *
 *  Created on: 30 сент. 2025?г.
 *      Author: farid
 */

#include "clockScreen.h"
#include "math.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "lvgl.h"
#include "mainPanel.h"
#include "header.h"
#include "settings.h"

#include "time.h"

LV_IMG_DECLARE(clockFace);
LV_IMG_DECLARE(hourHand);
LV_IMG_DECLARE(minHand);
LV_IMG_DECLARE(secHand);

static lv_timer_t *clockTimer;
static lv_obj_t *imgHourHand, *imgMinHand, *imgSecHand;

static void update_clock_cb(lv_timer_t *timer) {
    time_t now = time(NULL);
    struct tm *tm_now = localtime(&now);

    int hour_angle = ((tm_now->tm_hour % 12) * 30) + (tm_now->tm_min / 2); // 360/12
    int minute_angle = tm_now->tm_min * 6;
    int second_angle = tm_now->tm_sec * 6;

    lv_img_set_angle(imgHourHand, hour_angle * 10);   // LVGL: угол в 0.1°
    lv_img_set_angle(imgMinHand, minute_angle * 10);
    lv_img_set_angle(imgSecHand, second_angle * 10);
}

static void clock_panel_cleanup_cb(lv_event_t *e) {
    if (clockTimer) {
        lv_timer_del(clockTimer);
        clockTimer = NULL;
    }
}

void createClockScrreen() {
    destroyMainPanel();
    lv_obj_t *panel = createMainPanel();
    headerSetText("***Clock***");
    lv_obj_set_style_bg_color(panel,lv_color_make(90, 90, 120), LV_PART_MAIN);


    lv_obj_t *imgFace = lv_img_create(panel);
    lv_img_set_src(imgFace, &clockFace);
    lv_obj_center(imgFace);


    int pivotX, pivotY;
    int panel_center_x = MAIN_PANEL_WIDTH / 2;
    int panel_center_y = MAIN_PANEL_HEIGHT / 2;

    imgHourHand = lv_img_create(panel);
    lv_img_set_src(imgHourHand, &hourHand);
    pivotX = 30;
    pivotY = 123;
    lv_img_set_pivot(imgHourHand, pivotX, pivotY);
    lv_obj_set_pos(imgHourHand,
        panel_center_x - pivotX,
        panel_center_y - pivotY
    );

    imgMinHand = lv_img_create(panel);
    lv_img_set_src(imgMinHand, &minHand);
    pivotX = 23;
    pivotY = 146;
    lv_img_set_pivot(imgMinHand, pivotX, pivotY);
    lv_obj_set_pos(imgMinHand,
        panel_center_x - pivotX,
        panel_center_y - pivotY
    );

    imgSecHand = lv_img_create(panel);
    lv_img_set_src(imgSecHand, &secHand);
    pivotX = 5;
    pivotY = 146;
    lv_img_set_pivot(imgSecHand, pivotX, pivotY);
    lv_obj_set_pos(imgSecHand,
        panel_center_x - pivotX,
        panel_center_y - pivotY
    );

    update_clock_cb(clockTimer);

    // Запустить таймер обновления стрелок
    clockTimer = lv_timer_create(update_clock_cb, 1000, NULL);

    // Добавляем обработчик события удаления
    lv_obj_add_event_cb(panel, clock_panel_cleanup_cb, LV_EVENT_DELETE, NULL);

}

