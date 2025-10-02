/*
 * settings.h
 *
 *  Created on: 20 сент. 2025?г.
 *      Author: farid
 */

#ifndef MAIN_INC_SETTINGS_H_
#define MAIN_INC_SETTINGS_H_

#define LCD_H_RES 320
#define LCD_V_RES 480
#define HEADER_HEIGHT 30
#define FOOTER_HEIGHT 40
#define MAIN_PANEL_HEIGHT (LCD_V_RES - HEADER_HEIGHT - FOOTER_HEIGHT)
#define MAIN_PANEL_WIDTH LCD_H_RES
#define MAIN_PANEL_BG_COLOR lv_color_make(10, 10 ,10)
#define HEADER_COLOR lv_color_make(20, 20, 70)
#define FOOTER_COLOR HEADER_COLOR




#endif /* MAIN_INC_SETTINGS_H_ */
