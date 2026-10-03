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
 * @file include/ldr_driver.h
 *
 * @brief Interface para interagir com o sensor LDR.
 *
 * @author Vinicius Silva <silva.viniciusr@gmail.com>
 *
 * @date 02 de outubro de 2026
 */

#pragma once

#include "esp_err.h"

/**
 * @brief Solicita inicializacao do driver do sensor LDR
 *
 * @return
 *    - ESP_OK (0):            Success.
 *    - ESP_FAIL:              Falha ao criar a tarefa DHT11_D.
 *    - ESP_ERR_INVALID_STATE: Driver ja encontra-se inicializado.
 *
 */
esp_err_t LdrInit(void);

/**
 * @brief Solicita leitura do sensor LDR
 *
 * @param voltage ponteiro para um inteirno no qual
 *                o resultado da leitura sera gravado.
 *
 * @return
 *    - ESP_OK (0): Success.
 *    - ESP_ERR_INVALID_ARG:   voltage = NULL.
 *    - ESP_ERR_INVALID_STATE: Driver nao inicializado.
 *    - ESP_ERR_TIMEOUT:       Driver encontra-se ocupado. Tente novamente.
 *
 */
esp_err_t LdrRead(int *voltage);
