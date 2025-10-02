/*
 * GUI.c
 *
 *  Created on: 29 сент. 2025?г.
 *      Author: farid
 */

#include "GUI.h"
#include "lvgl.h"

#include <stdio.h>
#include "footer.h"
#include "header.h"
#include "mainScreen.h"
#include "styles.h"





void GUI() {
    initSyles();

    createHeader();
    createFooter();
    createMainScreen();
}

