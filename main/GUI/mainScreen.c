/*
 * mainScreen.c
 *
 *  Created on: 30 сент. 2025?г.
 *      Author: farid
 */


#include "stdio.h"
#include "mainScreen.h"
#include "exampleScreen.h"
#include "clockScreen.h"
#include "lvgl.h"
#include "settings.h"
#include "mainPanel.h"
#include "header.h"
#include "styles.h"


LV_IMAGE_DECLARE(clock);
LV_IMAGE_DECLARE(example);
LV_IMAGE_DECLARE(settings);

static void btnExampleEventCB(lv_event_t *e) {
    createExampleScreen();
}

static void btnClockEventCB(lv_event_t *e) {
    createClockScrreen();
}

void createMainScreen() {
    destroyMainPanel();

    headerSetText("***Main screen***");

    lv_obj_t *panel = createMainPanel();
    lv_obj_set_style_pad_all(panel, 5, LV_PART_MAIN);
    lv_obj_set_flex_flow(panel, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(panel, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);

    lv_obj_t *butExample = lv_button_create(panel);
    lv_obj_add_style(butExample, getStyleBtnIcon(), LV_PART_MAIN);
    lv_obj_set_flex_align(butExample, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_t *imgExample = lv_image_create(butExample);
    lv_img_set_src(imgExample, &example);
    lv_obj_t *labelExample = lv_label_create(butExample);
    lv_label_set_text(labelExample, "Example");
    lv_obj_add_event_cb(butExample, btnExampleEventCB, LV_EVENT_CLICKED, NULL);

    lv_obj_t *butClock = lv_button_create(panel);
    lv_obj_add_style(butClock, getStyleBtnIcon(), LV_PART_MAIN);
    lv_obj_set_flex_align(butClock, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_t *imgClock = lv_image_create(butClock);
    lv_img_set_src(imgClock, &clock);
    lv_obj_t *labelClock = lv_label_create(butClock);
    lv_label_set_text(labelClock, "Clock");
    lv_obj_add_event_cb(butClock, btnClockEventCB, LV_EVENT_CLICKED, NULL);

    lv_obj_t *butSettings = lv_button_create(panel);
    lv_obj_add_style(butSettings, getStyleBtnIcon(), LV_PART_MAIN);
    lv_obj_set_flex_align(butSettings, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_t *imgSettings = lv_image_create(butSettings);
    lv_img_set_src(imgSettings, &settings);
    lv_obj_t *labelSettings = lv_label_create(butSettings);
    lv_label_set_text(labelSettings, "Settings");
}

