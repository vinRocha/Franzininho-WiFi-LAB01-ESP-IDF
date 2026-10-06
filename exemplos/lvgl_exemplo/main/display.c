#include "display.h"

#include "driver/i2c_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"   /* esp_lcd_new_panel_ssd1306(), config type */
#include "esp_lvgl_port.h"
#include "esp_log.h"

static const char *s_TAG = "display";

static esp_lcd_panel_handle_t s_panel = NULL;

lv_display_t *oled_init(void)
{
    ESP_LOGI(s_TAG, "Init I2C bus (SDA=%d SCL=%d @ %d Hz)",
             OLED_SDA_GPIO, OLED_SCL_GPIO, OLED_I2C_HZ);

    i2c_master_bus_handle_t bus = NULL;
    const i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = -1,                     /* auto-select a free port */
        .sda_io_num = OLED_SDA_GPIO,
        .scl_io_num = OLED_SCL_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    ESP_LOGI(s_TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io = NULL;
    const esp_lcd_panel_io_i2c_config_t io_cfg = {
        .dev_addr = OLED_I2C_ADDR,
        .scl_speed_hz = OLED_I2C_HZ,
        .control_phase_bytes = 1,           /* SSD1306: control phase */
        .lcd_cmd_bits = 8,                  /* SSD1306 */
        .lcd_param_bits = 8,                /* SSD1306 */
        .dc_bit_offset = 6,                 /* SSD1306 */
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(bus, &io_cfg, &io));

    ESP_LOGI(s_TAG, "Install SSD1306 panel");
    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = -1,               /* GPIO_NUM_NC if RESET is tied */
        .bits_per_pixel = 1,                /* monochrome */
    };
    esp_lcd_panel_ssd1306_config_t ssd1306_cfg = {
        .height = OLED_V_RES,               /* 64 (or 32) */
    };
    panel_cfg.vendor_config = &ssd1306_cfg;

    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io, &panel_cfg, &s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel, true));

    ESP_LOGI(s_TAG, "Init LVGL port");
    const lvgl_port_cfg_t port_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    ESP_ERROR_CHECK(lvgl_port_init(&port_cfg));

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io,
        .panel_handle = s_panel,
        /* Monochrome REQUIRES a full-screen buffer. */
        .buffer_size = OLED_H_RES * OLED_V_RES,
        .double_buffer = true,
        .hres = OLED_H_RES,
        .vres = OLED_V_RES,
        .monochrome = true,
        .color_format = LV_COLOR_FORMAT_I1,
        .rotation = {
            .swap_xy = false,
            .mirror_x = true,
            .mirror_y = true,
        },
        .flags = {
            .swap_bytes = false,
            .sw_rotate = false,
        },
    };

    lv_display_t *disp = lvgl_port_add_disp(&disp_cfg);
    if (disp == NULL) {
        ESP_LOGE(s_TAG, "lvgl_port_add_disp failed");
        return NULL;
    }
    ESP_LOGI(s_TAG, "Display ready (%dx%d, I1)", OLED_H_RES, OLED_V_RES);
    return disp;
}

esp_lcd_panel_handle_t oled_get_panel(void)
{
    return s_panel;
}
