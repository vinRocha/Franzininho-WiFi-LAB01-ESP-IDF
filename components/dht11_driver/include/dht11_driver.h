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

/* Estrutura de dado de leitura do sensor */
typedef struct {
  float relative_humidity;
  float temperature;
} dht11_data_t;

/**
 * @brief Solicita inicializacao do driver do sensor DHT11
 *
 * @return
 *    - ESP_OK (0): Success.
 *    - ESP_FAIL: Falha ao criar a tarefa DHT11_D.
 *    - ESP_ERR_INVALID_STATE: Driver ja encontra-se inicializado.
 *
 */
esp_err_t Dht11Init(void);

/**
 * @brief Solicita atualizacao de leitura do sensor
 *
 * @return
 *    - ESP_OK (0): Success.
 *    - ESP_ERR_INVALID_STATE: Driver nao inicializado.
 *
 */
esp_err_t Dht11Update(void);

/**
 * @brief Escreve ultima leitura valida do sensor
 *
 * @param dht11_data ponteiro para uma estrutura dht11_data_t no qual
 *                   o resultado da leitura sera gravado.
 *
 * @return
 *    - ESP_OK (0): Success.
 *    - ESP_ERR_NOT_FINISHED:  Leitura em andamento, tente novamente.
 *    - ESP_ERR_INVALID_STATE: Driver nao inicializado.
 *
 */
esp_err_t Dht11Read(dht11_data_t *dht11_data);
