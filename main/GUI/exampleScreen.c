/*
 * exampleScreen.c
 *
 *  Created on: 30 сент. 2025?г.
 *      Author: farid
 */


#include "stdio.h"
#include "exampleScreen.h"
#include "lvgl.h"
#include "settings.h"
#include "mainPanel.h"
#include "header.h"

static lv_obj_t * label;

static void btn_event_cb(lv_event_t * e) {
    lv_obj_t * btn = lv_event_get_target(e);
    lv_obj_t * lbl = lv_obj_get_child(btn, 0); // предполагаем, что label Ч первый дочерний элемент
    const char * text = lv_label_get_text(lbl);
    printf("Button pressed: %s\n", text);
    fflush(stdout);


    lv_label_set_text(label, text); // мен€ем надпись сверху
}


void createExampleScreen() {
    destroyMainPanel();
    headerSetText("***Example Screen***");

    lv_obj_t *panel = createMainPanel();


    label = lv_label_create(getMainPanel());
    lv_label_set_text(label, "Hello LVGL!");
    lv_obj_set_style_text_color(label, lv_color_white(), LV_PART_MAIN);
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 10);

    const lv_color_t colors[3] = { lv_palette_main(LV_PALETTE_RED), lv_palette_main(LV_PALETTE_GREEN), lv_palette_main(LV_PALETTE_BLUE) };
    const char *texts[3] = { "Red", "Green", "Blue" };

    for (int i = 0; i < 3; i++) {
        lv_obj_t *btn = lv_button_create(lv_screen_active());
        lv_obj_set_size(btn, 60, 40);
        lv_obj_set_style_bg_color(btn, colors[i], 0);
        lv_obj_align(btn, LV_ALIGN_CENTER, (i - 1) * 80, 40);

        lv_obj_t *lbl = lv_label_create(btn);
        lv_label_set_text(lbl, texts[i]);
        lv_obj_center(lbl);

        lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_CLICKED, NULL);
    }
}

