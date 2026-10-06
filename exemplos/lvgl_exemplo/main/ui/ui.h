#pragma once

#include "lvgl.h"

/*
 * Build the Module 01 demo ("Hello, LVGL!") on the given display.
 *
 * Call only while holding the LVGL port lock (lvgl_port_lock()).
 * Later modules replace/extend the body of this file.
 */
void ui_init(lv_display_t *disp);