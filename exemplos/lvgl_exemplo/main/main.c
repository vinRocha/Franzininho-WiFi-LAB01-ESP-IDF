/**
 * SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2026 Vinicius Silva <silva.viniciusr@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file main.c
 *
 * @brief Aplicacao para uso do LVGL com display ssd1306
 *
 * @author Vinicius Silva <silva.viniciusr@gmail.com>
 *
 * @date 04 de outubro de 2026
 */

#include "esp_log.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "display.h"
#include "ui.h"

static const char *s_TAG = "app";

void app_main(void)
{
  /* Aguarda 2 segundos para finalizacao de inicializao do USB_CDC */
  vTaskDelay(pdMS_TO_TICKS(2000));

  lv_display_t *disp = oled_init();
  if (disp == NULL) {
    ESP_LOGE(s_TAG, "OLED init failed");
    return;
  }

  /* LVGL is not thread-safe: build the UI under the port lock. */
  if (lvgl_port_lock(0)) {
    ui_init(disp);
    lvgl_port_unlock();
  }

  ESP_LOGI(s_TAG, "UI ready");
}
