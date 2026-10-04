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
 * @file rgb_led_driver.h
 *
 * @brief Interface para interagir com o LED RGB via periferico LEDC (PWM).
 *
 * @author Vinicius Silva <silva.viniciusr@gmail.com>
 *
 * @date 03 de outubro de 2026
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"

/**
 * @brief Solicita inicialização do driver RGB.
 *
 * Cria a tarefa RGB_LED_D, que é responsável por configurar o timer
 * LEDC (timer 0) os canais LEDC (channel 0, channel 1 e channel 2)
 * e por processar os comandos enviados pelas funções públicas.
 *
 * @return
 *    - ESP_OK (0): Success.
 *    - ESP_FAIL: Falha ao criar a tarefa RGB_LED_D.
 *    - ESP_ERR_NOT_ALLOWED: Driver já encontra-se inicializado.
 */
esp_err_t RgbLedInit(void);

/**
 * @brief Configura a intensidade de brilho de cada LED.
 *
 *
 * @param r valor de brilho para o led vermelho (0 - 255)
 *
 * @param g valor de brilho para o led verde    (0 - 255)
 *
 * @param b valor de brilho para o led azul     (0 - 255)
 *
 * @return
 *    - ESP_OK (0): Success.
 *    - ESP_ERR_INVALID_STATE: Driver não inicializado.
 *    - ESP_ERR_TIMEOUT:       Driver encontra-se ocupado. Tente novamente.
 */
esp_err_t RgbLedSet(uint8_t r, uint8_t g, uint8_t b);
