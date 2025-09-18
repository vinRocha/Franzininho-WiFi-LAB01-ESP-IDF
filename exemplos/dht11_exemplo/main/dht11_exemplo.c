/**
 * SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2025 franzininho
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
 * @file dht11_exemplo.c
 *
 * @brief Aplicacao exemplo para coletar dados do sensor DHT11 e escreve-los no console.
 *
 * @author Vinicius Silva <silva.viniciusr@gmail.com>
 *
 * @date 14 de Maio de 2025
 */

#include <stdio.h>
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "dht11_driver.h"

static char *s_TAG = "app_main";

/**
 * @brief Loop principal
 *
 * Nao deve retornar.
 *
 */
void app_main(void) {

  esp_err_t rc;

  /* Estrutura para receber os dados lidos do sensor */
  dht11_data_t dht11_data;

  /* Inicia o driver do sensor DHT11 */
  if (Dht11Init() != ESP_OK) {
    ESP_LOGE(s_TAG, "Erro ao inicializar o driver DHT11...\n");
    for(;;){}
  }

  /* Loop infinito da aplicacao */
  for (;;) {

    //solicita atualizacao do sensor
    rc = Dht11Update();
    ESP_LOGI(s_TAG, "Dht11Update() rc: %d", rc);

    //solicita leitura atualizada do sensor e caso tenha sucesso na leitura, imprime o resultado no console.
    rc = Dht11Read(&dht11_data);
    ESP_LOGI(s_TAG, "Dht11Read() rc: %d", rc);
    if (!rc) {
      fprintf(stdout, "HuRe: %.2f\nTemp: %.2f\n\n",
              dht11_data.relative_humidity, dht11_data.temperature);
    }

    vTaskDelay(3000 / portTICK_PERIOD_MS);
  }

  /* Nao deve chegar aqui!! */
  return;
}
