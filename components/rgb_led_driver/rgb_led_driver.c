/**
 * SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2026 Franzininho
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
 * @file rgb_led_driver.c
 *
 * @brief Implementacao do driver para interagir com os LEDs RGB.
 *
 * @author Vinicius Silva <silva.viniciusr@gmail.com>
 *
 * @date 03 de outubro de 2026
 */

#include "esp_log.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "hal/ledc_types.h"
#include "rgb_led_driver.h"

#define R_GPIO                 CONFIG_RGB_LED_R_GPIO
#define G_GPIO                 CONFIG_RGB_LED_G_GPIO
#define B_GPIO                 CONFIG_RGB_LED_B_GPIO
#define R_CHANNEL              LEDC_CHANNEL_0
#define G_CHANNEL              LEDC_CHANNEL_1
#define B_CHANNEL              LEDC_CHANNEL_2
#define RGB_LED_FREQ           (5000)
#define LEDC_DUTY_RES          LEDC_TIMER_8_BIT
#define LEDC_DUTY_MAX          (1U << LEDC_DUTY_RES)
#define RGB_LED_TIMEOUT_MS     (100)

static const char *s_TAG = "RGB_LED_D";

struct driver_ctx
{
  uint8_t initialized;
/* Variavel para verificar possivel erro */
  esp_err_t rc;
};

static struct driver_ctx s_dctx = {0};

/* Mutex para indicar se driver encontra-se ocupado
 * Uma vez inicializado, esse mutex nao pode ser desalocado
 */
static SemaphoreHandle_t s_dmutex = NULL;

esp_err_t RgbLedInit(void)
{
  if (s_dctx.initialized)
  {
    return ESP_ERR_NOT_ALLOWED;
  }

  if (!s_dmutex)
  {
    s_dmutex = xSemaphoreCreateMutex();
    if (!s_dmutex)
    {
      ESP_LOGE(s_TAG, "Erro ao adiquirir o mutex. Tente iniciar o driver novamente");
      return ESP_ERR_NO_MEM;
    }
  }
  while(!xSemaphoreTake(s_dmutex, portMAX_DELAY))
    continue;

  ledc_timer_config_t ledc_timer_cfg = {
    .speed_mode       = LEDC_LOW_SPEED_MODE,
    .timer_num        = LEDC_TIMER_0,
    .duty_resolution  = LEDC_DUTY_RES,
    .freq_hz          = RGB_LED_FREQ,
    .clk_cfg          = LEDC_AUTO_CLK
  };
  s_dctx.rc = ledc_timer_config(&ledc_timer_cfg);
  if (s_dctx.rc) return s_dctx.rc;

  /* Caso algum erro aconteca com a configuracao dos canais... */
  ledc_timer_cfg.deconfigure = 1;

  ledc_channel_config_t ledc_channel_cfg = {0};
  ledc_channel_cfg.speed_mode = LEDC_LOW_SPEED_MODE;
  ledc_channel_cfg.timer_sel  = LEDC_TIMER_0;
  ledc_channel_cfg.intr_type  = LEDC_INTR_DISABLE;
  ledc_channel_cfg.duty       = 0;

  ledc_channel_cfg.channel  = R_CHANNEL;
  ledc_channel_cfg.gpio_num = R_GPIO;
  s_dctx.rc = ledc_channel_config(&ledc_channel_cfg);
  if (s_dctx.rc)
  {
    ledc_timer_config(&ledc_timer_cfg);
    return s_dctx.rc;
  }

  ledc_channel_cfg.channel  = G_CHANNEL;
  ledc_channel_cfg.gpio_num = G_GPIO;
  s_dctx.rc = ledc_channel_config(&ledc_channel_cfg);
  if (s_dctx.rc)
  {
    ledc_timer_config(&ledc_timer_cfg);
    return s_dctx.rc;
  }

  ledc_channel_cfg.channel  = B_CHANNEL;
  ledc_channel_cfg.gpio_num = B_GPIO;
  s_dctx.rc = ledc_channel_config(&ledc_channel_cfg);
  if (s_dctx.rc)
  {
    ledc_timer_config(&ledc_timer_cfg);
    return s_dctx.rc;
  }

  s_dctx.initialized = 1;
  xSemaphoreGive(s_dmutex);
  return ESP_OK;
}

esp_err_t RgbLedSet(uint8_t r, uint8_t g, uint8_t b)
{
  if (!s_dmutex)
    return ESP_ERR_INVALID_STATE;

  if (!xSemaphoreTake(s_dmutex, pdMS_TO_TICKS(RGB_LED_TIMEOUT_MS)))
    return ESP_ERR_TIMEOUT;

  if (!s_dctx.initialized)
  {
    xSemaphoreGive(s_dmutex);
    return ESP_ERR_INVALID_STATE;
  }

  ledc_set_duty(LEDC_LOW_SPEED_MODE, R_CHANNEL, r);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, R_CHANNEL);

  ledc_set_duty(LEDC_LOW_SPEED_MODE, G_CHANNEL, g);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, G_CHANNEL);

  ledc_set_duty(LEDC_LOW_SPEED_MODE, B_CHANNEL, b);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, B_CHANNEL);

  xSemaphoreGive(s_dmutex);
  return ESP_OK;
}
