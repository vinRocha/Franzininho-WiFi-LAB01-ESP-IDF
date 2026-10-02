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
 * @file input_exemplo.c
 *
 * @brief Aplicacao exemplo para ler o estado dos botoes e escreve-los no console 15x por segundo.
 *
 * @author Vinicius Silva <silva.viniciusr@gmail.com>
 *
 * @date 14 de Maio de 2025
 */

#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "input_driver.h"

static char *s_TAG = "app_main";

/**
 * @brief Loop principal
 *
 * Pode retornar em caso de erro.
 *
 */
void app_main(void)
{
  esp_err_t rc;

  /* Estrutura para receber os estados dos botoes */
  input_data_t input_data;

  /* Inicia o driver de input */
  if (InputInit() != ESP_OK)
  {
    ESP_LOGE(s_TAG, "Erro ao inicializar o driver de input...\n");
    return;
  }

  /* Loop infinito da aplicacao */
  for (;;)
  {
    //realiza a leitura do estado dos botoes imprime o resultado no console.
    rc = InputRead(&input_data);
    /* Limpa e reseta o terminal. */
    fprintf(stdout, "\033[2J\033[H");
    ESP_LOGI(s_TAG, "InputRead() rc: %d", rc);
    if (!rc) {
      fprintf(stdout, "BT1: %s\nBT2: %s\nBT3: %s\nBT4: %s\nBT5: %s\nBT6: %s\nCHANGED: %s\n",
              input_data.bt1 ? "unpressed" : "pressed",
              input_data.bt2 ? "unpressed" : "pressed",
              input_data.bt3 ? "unpressed" : "pressed",
              input_data.bt4 ? "unpressed" : "pressed",
              input_data.bt5 ? "unpressed" : "pressed",
              input_data.bt6 ? "unpressed" : "pressed",
              input_data.changed ? "true" : "false");
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }

  /* Nao deve chegar aqui!! */
  return;
}
