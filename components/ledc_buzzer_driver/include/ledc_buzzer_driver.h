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
 * @file include/ledc_buzzer_driver.h
 *
 * @brief Interface para interagir com o buzzer via periférico LEDC (PWM).
 *
 * Substitui o antigo driver buzzer_driver que usava DAC_COSINE. Usa
 * LEDC para controle de frequência suave e sem reconfiguração
 * de periféricos.
 *
 * @author Vinicius Silva <silva.viniciusr@gmail.com>
 *
 * @date 01 de Outubro de 2025
 */

#pragma once

#include "esp_err.h"

/**
 * @brief Solicita inicialização do driver LEDC do buzzer.
 *
 * Configura o timer LEDC (timer 0) e canal LEDC (channel 0, GPIO 17).
 * Não cria tarefas — usa apenas chamadas diretas ao periférico.
 *
 * @return
 *    - ESP_OK (0): Success.
 *    - ESP_ERR_INVALID_STATE: Driver já inicializado.
 *    - ESP_FAIL: Falha na configuração do timer ou canal LEDC.
 */
esp_err_t LEDBuzzerInit(void);

/**
 * @brief Liga e desliga o buzzer com determinada frequência.
 *
 * @param value 0 = OFF (duty = 0)
 *               > 0 = ON (duty = ~50%)
 * @param freq  Frequência em Hertz da onda PWM gerada
 *
 * @return
 *    - ESP_OK (0): Success.
 *    - ESP_ERR_INVALID_STATE: Driver não inicializado.
 *    - ESP_FAIL: Erro ao configurar a frequência LEDC.
 */
esp_err_t LEDBuzzerSet(char value, int freq);

/**
 * @brief Configura o buzzer para tocar periodicamente com determinado duty cycle.
 *
 * Usa PWM direto via LEDC com período em ms e ciclo de trabalho (duty_cycle)
 * entre 0-100 %. Se period == 0 o buzzer será desligado.
 *
 * @param period     Período total do pulso em milissegundos
 * @param duty_cycle Duty cycle em percentual (0 a 100). Valores > 100 são truncados para 100.
 *
 * caso period = 0 o buzzer será desligado.
 * caso duty_cycle > 100, será truncado em 100.
 *
 * @return
 *    - ESP_OK (0): Success.
 *    - ESP_ERR_INVALID_STATE: Driver não inicializado.
 */
esp_err_t LEDBuzzerPulse(unsigned period, unsigned duty_cycle);
