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
 * @file luz_automatica.c
 *
 * @brief Aplicacao exemplo para ler o sensor LDR e controlar o brilho do LED RGB
 * de acordo com a intensidade de luz do ambiente.
 *
 * @author Vinicius Silva <silva.viniciusr@gmail.com>
 *
 * @date 04 de outubro de 2026
 */

#include <stdio.h>
#include "esp_log.h"
#include  "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rgb_led_driver.h"
#include "ldr_driver.h"

static char *s_TAG = "app_main";

#define MAX_VOLTAGE   (2530)
#define MAX_INTENSITY ( 255)

static esp_err_t init_drivers(void)
{
  /* Inicia o driver de RGB */
  if (RgbLedInit() != ESP_OK)
  {
    ESP_LOGE(s_TAG, "Erro ao inicializar o driver dos LEDs RGB...\n");
    return ESP_ERR_INVALID_STATE;
  }

  /* Inicia o driver de LDR */
  if (LdrInit() != ESP_OK)
  {
    ESP_LOGE(s_TAG, "Erro ao inicializar o driver do LDR...\n");
    return ESP_ERR_INVALID_STATE;
  }

  return ESP_OK;
}

/**
 * @brief Loop principal
 *
 * Pode retornar em caso de erro.
 *
 */
void app_main(void)
{
  int voltage;
  uint8_t intensity;
  float helper;

  /* Aguarda 2 segundos para finalizacao de inicializao do HW */
  vTaskDelay(pdMS_TO_TICKS(2000));

  /* Inicia os drivers da aplicacao */
  if (init_drivers() != ESP_OK)
  {
    esp_restart();
    return;
  }

  /* Loop infinito da aplicacao */
  for (;;)
  {
    LdrRead(&voltage);
    helper = (1 - (float) voltage / MAX_VOLTAGE) * MAX_INTENSITY;
    intensity = helper;
    RgbLedSet(intensity, intensity, intensity);
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  /* Nao deve chegar aqui!! */
  return;
}
