/*
 * SPDX-License-Identifier: MIT
 *
 */

/**
 * @file include/buzzer_driver.h
 *
 * @brief Interface para interagir com o buzzer.
 *
 * @author Vinicius Silva <silva.viniciusr@gmail.com>
 *
 * @date 29 de Maio de 2025
 */

#pragma once

#include "esp_err.h"

/**
 * @brief Solicita inicializacao do driver do buzzer.
 *
 * @return
 *    - ESP_OK (0): Success.
 *    - ESP_FAIL: Falha ao criar a tarefa BUZZER_D.
 *    - ESP_ERR_NOT_ALLOWED: Driver ja encontra-se inicializado.
 *
 */
esp_err_t BuzzerInit(void);

/**
 * @brief Liga e desliga o buzzer.
 *
 * @param value: 0 = OFF
 *               1 = ON
 *
 * @return
 *    - ESP_OK (0): Success.
 *    - ESP_ERR_INVALID_STATE: Driver nao inicializado.
 *
 */
esp_err_t BuzzerSet(char value);

/**
 * @brief Configura o buzzer para tocar periodicamente com determinado duty cycle.
 *
 * @param period Periodo em MS
 * @param duty_cycle Duty cycle 0-100
 *
 * caso period = 0 o buzzer sera desligado.
 * caso duty_cycle > 100, sera truncado em 100.
 *
 * @return
 *    - ESP_OK (0): Success.
 *    - ESP_ERR_INVALID_STATE: Driver nao inicializado.
 *
 */
esp_err_t BuzzerBeat(unsigned period, unsigned duty_cycle);
