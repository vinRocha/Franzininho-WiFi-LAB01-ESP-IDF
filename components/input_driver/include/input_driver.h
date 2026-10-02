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
 * @file include/input_driver.h
 *
 * @brief Interface para ler o estaado dos botoes da Franzininho WIFI-LAB01.
 *
 * @author Vinicius Silva <silva.viniciusr@gmail.com>
 *
 * @date 01 de outubro de 2026
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"

/* Estrutura de leitura dos botoes */
typedef struct {
  uint8_t bt1      :1;
  uint8_t bt2      :1;
  uint8_t bt3      :1;
  uint8_t bt4      :1;
  uint8_t bt5      :1;
  uint8_t bt6      :1;
  uint8_t changed  :1;
  uint8_t reserved :1;
} input_data_t;

/**
 * @brief Solicita inicializacao do driver de input
 *
 * @return
 *    - ESP_OK (0):            Success.
 *    - ESP_FAIL:              Falha ao criar a tarefa INPUT_D.
 *    - ESP_ERR_INVALID_STATE: Driver ja encontra-se inicializado.
 *
 */
esp_err_t InputInit(void);

/**
 * @brief Realiza leitura do estado dos botoes com deboucing via SW.
 *
 * @param input_data ponteiro para uma estrutura input_data_t na
 *                   qual o estado atual dos botões sera gravado.
 *
 * @return
 *    - ESP_OK (0): Success.
 *    - ESP_ERR_INVALID_ARG:   input_data = NULL.
 *    - ESP_ERR_INVALID_STATE: Driver nao inicializado.
 *    - ESP_ERR_TIMEOUT:       Driver encontra-se ocupado. Tente novamente.
 *
 */
esp_err_t InputRead(input_data_t *input_data);
