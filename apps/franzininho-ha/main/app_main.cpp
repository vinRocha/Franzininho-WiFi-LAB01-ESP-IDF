/**
 * SPDX-License-Identifier: GPLv2
 *
 * Copyright (C) 2025  Vinicius Silva <silva.viniciusr@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/**
 * @file app_main.cpp
 *
 * @brief franzininho-ha application.
 *
 * @author Vinicius Silva <silva.viniciusr@gmail.com>
 *
 * @date July 12th 2025
 *
 */

#include <cstdint>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "protocol_examples_common.h"
#include "mqtt_manager.h"
#include "ha_switch.h"

static const char *s_TAG = "main_app";

static esp_err_t s_BoardInit();
static void      s_led_cb(HaSwitch *switch_p);

extern "C" void app_main()
{

  if (s_BoardInit() != ESP_OK)
  {
    esp_restart();
    for (;;) {}
  }

  constexpr int num_switches {6};
  HaSwitch switches[num_switches] {
    HaSwitch(false),
    HaSwitch(false),
    HaSwitch(false),
    HaSwitch(false),
    HaSwitch(false),
    HaSwitch(true, s_led_cb)
  };

  for (auto &my_switch : switches)
  {
    ESP_ERROR_CHECK(my_switch.Connect());
  }

  /* We call reset() to synchronize the state of the physical
   * LED with the application and MQTT integration */
  ESP_ERROR_CHECK(switches[5].reset());

  /* TODO: Replace below code with LVGL */
#if 0
  int selection = 0;
  s_DrawBaseGui();
  ssd1306_display_text(s_app_cfg.ssd1306, selection + 1, " -> ", 4, false);
  while(true) {
    uint32_t input = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    /* If input, process it. */
    if (input) {
      ESP_LOGI(s_TAG, "Received notification. Processing GPIO mask %#.8x.", input);
      ssd1306_display_text(s_app_cfg.ssd1306, selection + 1, "    ", 4, false);
      switch (input) {
        case (1LLU << 7) : //Button UP
          if (selection == 0)
            selection = (num_switches - 1);
          else
            --selection;
          break;
        case (1LLU << 4) : //Button DOWN
          ++selection;
          selection %= (num_switches);
          break;
        case (1LLU << 2) : //Button ENTER
          switches[selection].toggle();
          s_PlayAnimation(switches[selection].get());
          s_DrawBaseGui();
          break;
        default :
        ESP_LOGI(s_TAG, "Unknown function for GPIO mask: %#.8x", input);
      }
      ssd1306_display_text(s_app_cfg.ssd1306, selection + 1, " -> ", 4, false);
      //deboucing....
      vTaskDelay(180 / portTICK_PERIOD_MS);
      ulTaskNotifyTake(pdTRUE, 0);
    }
    else
      ESP_LOGI(s_TAG, "Notification not received. Trying again...");
  }
#endif
}

esp_err_t s_BoardInit() {

  esp_err_t rc;
  if ((rc = nvs_flash_init()))
    return rc;
  if ((rc = esp_netif_init()))
    return rc;
  if ((rc = esp_event_loop_create_default()))
    return rc;
  if ((rc = example_connect()))
    return rc;
  if ((rc = MqttInit()))
    return rc;
  vTaskDelay(3000 / portTICK_PERIOD_MS);
  return rc = MqttPublish("franzininho-wifi/status", "online", 0, 0, 1);
}

void s_led_cb(HaSwitch *switch_p) {

  gpio_set_level(c_led_gpio, switch_p->get());
}
