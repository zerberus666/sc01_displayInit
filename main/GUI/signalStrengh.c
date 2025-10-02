/*
 * signalStrengh.c
 *
 *  Created on: 21 сент. 2025?г.
 *      Author: farid
 */
#include "signalStrengh.h"
#include "lvgl.h"

lv_obj_t *create_signal_indicator(lv_obj_t *parent, int level, int width,
								  int height) {
	//  онтейнер дл€ индикатора
	lv_obj_t *container = lv_obj_create(parent);
	lv_obj_set_size(container, width, height);
	lv_obj_set_style_bg_opa(container, LV_OPA_30, LV_PART_MAIN);
	lv_obj_set_style_border_width(container, 0, LV_PART_MAIN);
	lv_obj_set_style_pad_row(container, 0, LV_PART_MAIN);
	lv_obj_set_style_pad_column(container, 0, LV_PART_MAIN);
	lv_obj_set_style_pad_all(container, 0, LV_PART_MAIN);
	lv_obj_set_style_radius(container, 0, LV_PART_MAIN);
	lv_obj_clear_flag(container, LV_OBJ_FLAG_SCROLLABLE);
	lv_obj_set_layout(container, LV_LAYOUT_FLEX);

	// Flex: горизонтально, выравнивание по низу и вправо
	lv_obj_set_flex_flow(container, LV_FLEX_FLOW_ROW);
	lv_obj_set_flex_align(container,
						  LV_FLEX_ALIGN_END, // вертикально вниз
						  LV_FLEX_ALIGN_END, // горизонтально вправо
						  LV_FLEX_ALIGN_CENTER);

	const int bar_count = 5;
	const int spacing = 2; // фиксированный
	const int total_spacing = spacing * (bar_count - 1);
	int bar_width = (width - total_spacing) / bar_count;
	//int bar_width = 1;
	if (bar_width <= 0)
		bar_width = 1;

	for (int i = 0; i < bar_count; i++) {
		lv_obj_t *bar = lv_bar_create(container);
		int bar_height = height * (i + 1) / bar_count;

		lv_obj_set_size(bar, bar_width, bar_height);
		lv_bar_set_range(bar, 0, 1);
		lv_bar_set_value(bar, 1, LV_ANIM_OFF);

		// ќтключаем layout и scroll у полоски
		lv_obj_set_layout(bar, LV_LAYOUT_NONE);
		lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);

		// —тилизаци€ LV_PART_MAIN (об€зательна€ дл€ стабильности)
		lv_obj_set_style_bg_opa(bar, LV_OPA_TRANSP, LV_PART_MAIN);
		lv_obj_set_style_border_width(bar, 0, LV_PART_MAIN);
		lv_obj_set_style_pad_all(bar, 0, LV_PART_MAIN);
		lv_obj_set_style_radius(bar, 0, LV_PART_MAIN);

		// ќтступ справа, кроме последней полоски
		if (i < bar_count - 1) {
		    //lv_obj_set_style_margin_left(bar, 0, LV_PART_MAIN);
		    //lv_obj_set_style_pad_left(bar, 0, LV_PART_MAIN);
			lv_obj_set_style_margin_right(bar, spacing , LV_PART_MAIN);
		} else {
		    //lv_obj_set_style_margin_left(bar, 0, LV_PART_MAIN);
			lv_obj_set_style_margin_right(bar, 0, LV_PART_MAIN);
		}

		// —тилизаци€ LV_PART_INDICATOR Ч видима€ часть полоски
		lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, LV_PART_INDICATOR);
		lv_obj_set_style_radius(bar, 0, LV_PART_INDICATOR);
		lv_obj_set_style_border_width(bar, 0, LV_PART_INDICATOR);
		lv_obj_set_style_pad_all(bar, 0, LV_PART_INDICATOR);

		lv_color_t active = lv_color_white();
		lv_color_t inactive = lv_color_make(80, 80, 80);
		lv_obj_set_style_bg_color(bar, i < level ? active : inactive,
								  LV_PART_INDICATOR);
	}

	return container;
}

void update_signal_indicator(lv_obj_t *container, int level) {
	uint32_t count = lv_obj_get_child_cnt(container);
	for (uint32_t i = 0; i < count; i++) {
		lv_obj_t *bar = lv_obj_get_child(container, i);
		lv_obj_set_style_bg_color(
			bar, i < level ? lv_color_white() : lv_color_make(80, 80, 80),
			LV_PART_MAIN);
	}
}
