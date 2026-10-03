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
 * @file include/ledc_buzzer_driver.h
 *
 * @brief Interface para interagir com o buzzer via periférico LEDC (PWM).
 *
 * Substitui o antigo driver buzzer_driver que usava DAC_COSINE. Usa LEDC
 * para controle de frequência e ciclo de trabalho. A configuração e o
 * acionamento do periférico são realizados pela tarefa LEDC_BUZZER_D,
 * de modo que as funções públicas não bloqueiam a aplicação.
 *
 * @author Vinicius Silva <silva.viniciusr@gmail.com>
 *
 * @date 03 de outubro de 2026
 */

#pragma once

#include "esp_err.h"

/**
 * @brief Solicita inicialização do driver LEDC do buzzer.
 *
 * Cria a tarefa LEDC_BUZZER_D, que é responsável por configurar o timer
 * LEDC (timer 0) e canal LEDC (channel 0, GPIO 17) e por processar os
 * comandos enviados pelas funções públicas.
 *
 * @return
 *    - ESP_OK (0): Success.
 *    - ESP_FAIL: Falha ao criar a tarefa LEDC_BUZZER_D.
 *    - ESP_ERR_NOT_ALLOWED: Driver já encontra-se inicializado.
 */
esp_err_t LedcBuzzerInit(void);

/**
 * @brief Liga e desliga o buzzer com determinada frequência.
 *
 * Envia o comando para a tarefa LEDC_BUZZER_D e retorna imediatamente.
 * Com value > 0 o buzzer passa a emitir um tom contínuo na frequência
 * informada; com value = 0 o buzzer é desligado (duty = 0).
 *
 * @param value 0 = OFF (duty = 0)
 *              > 0 = ON (duty = ~50%)
 * @param freq  Frequência em Hertz da onda PWM gerada (40 - 15625 Hz).
 *
 * @return
 *    - ESP_OK (0): Success.
 *    - ESP_ERR_INVALID_ARG:   Frequência fora do intervalo válido.
 *    - ESP_ERR_INVALID_STATE: Driver não inicializado.
 *    - ESP_ERR_TIMEOUT:       Driver encontra-se ocupado. Tente novamente.
 */
esp_err_t LedcBuzzerSet(char value, int freq);

/**
 * @brief Configura o buzzer para tocar periodicamente com determinado duty cycle.
 *
 * Envia o comando para a tarefa LEDC_BUZZER_D e retorna imediatamente. A
 * tarefa passa a ligar o buzzer por period_on ms e desligá-lo por period_off
 * ms, repetidamente, até que um novo comando seja recebido.
 *
 * @param period     Período total do pulso em milissegundos.
 * @param duty_cycle Duty cycle em percentual (0 a 100). Valores > 100 são truncados para 100.
 *
 * caso period = 0 o buzzer será desligado.
 * caso duty_cycle > 100, será truncado em 100.
 *
 * @return
 *    - ESP_OK (0): Success.
 *    - ESP_ERR_INVALID_STATE: Driver não inicializado.
 *    - ESP_ERR_TIMEOUT:       Driver encontra-se ocupado. Tente novamente.
 */
esp_err_t LedcBuzzerPulse(unsigned period, unsigned duty_cycle);