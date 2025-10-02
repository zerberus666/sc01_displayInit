/*
 * signalStrengh.h
 *
 *  Created on: 21 сент. 2025?г.
 *      Author: farid
 */

#ifndef MAIN_INC_SIGNALSTRENGH_H_
#define MAIN_INC_SIGNALSTRENGH_H_

#include "lvgl.h"
#include "stdbool.h"

lv_obj_t *create_signal_indicator(lv_obj_t *parent, int level, int width, int height);
void update_signal_indicator(lv_obj_t *container, int level);



#endif /* MAIN_INC_SIGNALSTRENGH_H_ */
