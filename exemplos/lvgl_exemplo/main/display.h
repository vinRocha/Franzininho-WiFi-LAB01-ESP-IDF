#pragma once

#include "lvgl.h"
#include "esp_err.h"
#include "esp_lcd_panel_ops.h"

/* ---------------------------------------------------------------------------
 * Wiring -- EDIT to match your board / module.
 * ------------------------------------------------------------------------- */
#define OLED_H_RES        128
#define OLED_V_RES        64
#define OLED_I2C_ADDR     0x3C      /* 0x3D if the module's ADDR pin is high */
#define OLED_SDA_GPIO     8
#define OLED_SCL_GPIO     9
#define OLED_I2C_HZ       (400 * 1000)

/* ---------------------------------------------------------------------------
 * Initialise the SSD1306 over I2C and register it with esp_lvgl_port in
 * LV_COLOR_FORMAT_I1 monochrome mode.
 *
 * Returns the LVGL display handle, or NULL on failure.
 * Must be called once, before any UI is built.
 * ------------------------------------------------------------------------- */
lv_display_t *oled_init(void);

/* Panel handle, e.g. for sleep in Module 07 (esp_lcd_panel_disp_on_off). */
esp_lcd_panel_handle_t oled_get_panel(void);