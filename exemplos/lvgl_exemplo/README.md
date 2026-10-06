# learn_lvgl

Learning **LVGL 9.2** on an **ESP32-S2** with a **SSD1306 128×64 I2C OLED**, using
**ESP-IDF v5.5.5** and the Espressif Component Registry.

The full study plan (module by module, with snippets and milestones) lives in
[`.opencode/plan/`](.opencode/plan/README.md). This repository is the code that
grows alongside it — the current state is **Module 01**.

## Hardware

| Signal | GPIO | Notes |
|---|---|---|
| SDA | 8 | 4.7 kΩ pull-up to 3V3 recommended |
| SCL | 9 | 4.7 kΩ pull-up to 3V3 recommended |
| VCC | 3V3 | |
| GND | GND | |
| RESET | — | modules usually tie it; firmware uses `-1` |

Change the wiring in [`main/display.h`](main/display.h). The panel address is
`0x3C` by default (`0x3D` if the module's ADDR pin is high).

## Software baseline

- ESP-IDF **v5.5.5**
- `lvgl/lvgl` **~9.2.0** (Component Registry)
- `espressif/esp_lvgl_port` **^2**, monochrome mode (`LV_COLOR_FORMAT_I1`)
- In-tree `esp_lcd` SSD1306 panel driver (`esp_lcd_new_panel_ssd1306`)

## Build & flash

```bash
. $HOME/esp/esp-idf/export.sh     # adjust to your install
idf.py set-target esp32s2         # optional, CONFIG_IDF_TARGET is in sdkconfig.defaults
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

To exit the monitor: `Ctrl-]`.

## Layout

```
.
├── CMakeLists.txt
├── sdkconfig.defaults        # target + LVGL mono config (fonts, theme)
├── main/
│   ├── CMakeLists.txt
│   ├── idf_component.yml     # lvgl 9.2.0 + esp_lvgl_port (+ button in M05)
│   ├── main.c                # app_main
│   ├── display.c / display.h # I2C + SSD1306 + esp_lvgl_port (I1 monochrome)
│   └── ui/
│       ├── ui.h
│       └── ui_main.c         # Module 01 UI
└── docs/pitfalls.md
```

## Module 01 result

After flashing, the OLED shows:

```
      Hello, LVGL!
     128x64 SSD1306
```

Once that works, proceed to Module 02 in the plan.

## Rule of thumb

The display is 1 bpp and full-refresh over I2C: **every change redraws the whole
screen** (~1024 bytes). Keep object counts low and don't chase high frame rates.