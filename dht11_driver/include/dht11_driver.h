/*
 * SPDX-License-Identifier: MIT
 *
 */

/**
 * @file include/dht11_driver.h
 *
 * @brief Interface para interagir com o sensor DHT11.
 *
 * @author Vinicius Silva <silva.viniciusr@gmail.com>
 *
 * @date 14 de Maio de 2025
 */

#pragma once

#include "esp_err.h"

/* GPIO no qual o sensor encontra-se conectado */
#define DHT11_GPIO                      (15)

/* Estrutura de dado de leitura do sensor */
typedef struct {
  float relative_humidity;
  float temperature;
} dht11_data_t;

/**
 * @brief Loop principal da tarefa DHT11
 *
 * @param arg ponteiro para dados, nao utilizado.
 *
 * Exemplo:
 *
 *   xTaskCreate(Dht11Task, "DHT11", mDHT11_STACK_SIZE, NULL,
 *               mDHT11_PRIORITY, NULL);
 *
 */
void Dht11Task(void *pvParameters);

/**
 * @brief Solicita atualizacao de leitura do sensor
 *
 * @return
 *    - ESP_OK (0): Success
 *    - ESP_ERR_INVALID_STATE: Tarefa DHT11 nao esta executando.
 *                             Tentar criar a tarefa novamente com xTaskCreate.
 *
 */
esp_err_t Dht11Update(void);

/**
 * @brief Escreve ultima leitura valida do sensor
 *
 * @param dht11_data ponteiro para uma estrutura dht11_data_t aonde
 *                   o resultado da leitura sera salvo.
 *
 * @return
 *    - ESP_OK (0): Success
 *    - ESP_ERR_NOT_FINISHED:  Leitura em andamento, tente novamente.
 *    - ESP_ERR_INVALID_STATE: Tarefa DHT11 nao esta executando.
 *                             Tentar criar a tarefa novamente com xTaskCreate.
 *
 */
esp_err_t Dht11Read(dht11_data_t *dht11_data);
