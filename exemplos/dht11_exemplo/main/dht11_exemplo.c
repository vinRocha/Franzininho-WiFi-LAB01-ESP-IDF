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

/* Definicao de Tasks priorities. De acordo com a documentacao em 
 * https://docs.espressif.com/projects/esp-idf/en/v5.4.1/esp32s2/api-reference/system/freertos.html
 * Main task (app_main) tem prioridade 1: tskIDLE_PRIORITY + 1
 */
#define mAPP_PRIORITY              (tskIDLE_PRIORITY + 1)
#define mDHT11_PRIORITY            (tskIDLE_PRIORITY + 2)

/* Definicao de Tasks StackSize: minimal size + padding */
#define mDHT11_STACK_SIZE          4096 + 8 //por enquanto um valor alto.

/**
 * @brief Loop principal
 *
 * Nao deve retornar.
 *
 */
void app_main(void) {

  /*  Create DHT11 task */
  if (xTaskCreate(Dht11Task, "DHT11", mDHT11_STACK_SIZE, NULL,
                  mDHT11_PRIORITY, NULL) != pdPASS) {
    fprintf(stderr, "Erro criando a tarefa DHT11...\n");
    for (;;);
  }

  dht11_data_t dht11_data;

  for (;;) {

    Dht11Update(); //solicita atualizacao do sensor
    vTaskDelay(50 / portTICK_PERIOD_MS); //executa qualquer outra tarefa por 50ms
    if (!Dht11Read(&dht11_data)) { //recebe leitura atualizada do sensor e printa
                                   //resultado no console caso tenha sucesso na leitura
      fprintf(stdout, "HuRe: %.2f\nTemp: %.2f\n\n",
              dht11_data.relative_humidity, dht11_data.temperature);
    }

    vTaskDelay(3000 / portTICK_PERIOD_MS);
  }

  //Nao deve chegar aqui!!
  return;
}
