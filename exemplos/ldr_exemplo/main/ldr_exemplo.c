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
 * @file ldr_exemplo.c
 *
 * @brief Aplicacao exemplo para ler o valor do sensor LDR e escreve-lo no console 5x por segundo.
 *
 * @author Vinicius Silva <silva.viniciusr@gmail.com>
 *
 * @date 02 de outubro de 2026
 */

#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ldr_driver.h"

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
  int voltage;

  /* Aguarda 2 segundos para finalizacao de inicializao do HW */
  vTaskDelay(pdMS_TO_TICKS(2000));

  /* Inicia o driver de input */
  if (LdrInit() != ESP_OK)
  {
    ESP_LOGE(s_TAG, "Erro ao inicializar o driver de LDR...\n");
    return;
  }

  /* Loop infinito da aplicacao */
  for (;;)
  {
    rc = LdrRead(&voltage);
    ESP_LOGI(s_TAG, "LdrRead() rc: %d", rc);
    if (!rc)
      fprintf(stdout, "Tensao no sensor LDR: %dmV\n", voltage);

    vTaskDelay(200 / portTICK_PERIOD_MS);
    /* Limpa as duas ultimas linhas e escreve os dados novamente. */
    fprintf(stdout, "\r\033[2A\033[J");
  }

  /* Nao deve chegar aqui!! */
  return;
}
