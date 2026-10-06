#include "ui.h"

/* ---------------------------------------------------------------------------
 * Module 01 -- first screen.
 *
 * Keep this deliberately small: the goal is to prove the display pipeline
 * (panel + esp_lvgl_port + I1 monochrome) end to end. Modules 02+ grow this
 * into real screens.
 * ------------------------------------------------------------------------- */

void ui_init(lv_display_t *disp)
{
    lv_obj_t *scr = lv_display_get_screen_active(disp);
    lv_obj_clean(scr);                      /* start from an empty screen */

#if LV_USE_THEME_MONO
    /* Use the monochrome theme and the crisp 1-bit font. */
    lv_theme_t *theme = lv_theme_mono_init(disp, true, &lv_font_unscii_8);
    lv_display_set_theme(disp, theme);
#endif

    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "Franzininho");
    lv_obj_center(title);

    lv_obj_t *sub = lv_label_create(scr);
    lv_label_set_text(sub, "128x64 SSD1306");
    lv_obj_set_style_text_font(sub, &lv_font_unscii_8, 0);
    lv_obj_align(sub, LV_ALIGN_BOTTOM_MID, 0, -2);
}
