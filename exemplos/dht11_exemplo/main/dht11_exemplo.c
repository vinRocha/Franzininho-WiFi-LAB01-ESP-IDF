/*
 * SPDX-License-Identifier: MIT
 *
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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "dht11_driver.h"

/**
 * @brief Loop principal
 *
 * Nao deve retornar.
 *
 */
void app_main(void) {

  /* Estrutura para receber os dados lidos do sensor */
  dht11_data_t dht11_data;
  TaskHandle_t dht11_task;

  /* Inicia o driver do sensor DHT11 */
  if (Dht11Init(&dht11_task) != ESP_OK) {
    fprintf(stderr, "Erro ao inicializar o driver DHT11...\n");
    for(;;){}
  }

  /* Loop infinito da aplicacao */
  for (;;) {

    Dht11Update(); //solicita atualizacao do sensor

    vTaskDelay(50 / portTICK_PERIOD_MS); //executa qualquer outra tarefa por 50ms

    if (!Dht11Read(&dht11_data)) { //solicita leitura atualizada do sensor e caso tenha
                                   //sucesso na leitura, printa o resultado no console.
      fprintf(stdout, "HuRe: %.2f\nTemp: %.2f\n\n",
              dht11_data.relative_humidity, dht11_data.temperature);
    }

    fprintf(stdout, "uxTaskGetStackHighWaterMark2: %lu\n", uxTaskGetStackHighWaterMark2(dht11_task));

    vTaskDelay(3000 / portTICK_PERIOD_MS);
  }

  /* Nao deve chegar aqui!! */
  return;
}
