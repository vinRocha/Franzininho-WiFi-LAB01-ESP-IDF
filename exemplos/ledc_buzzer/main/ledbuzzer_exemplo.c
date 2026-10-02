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
 * @file ledbuzzer_exemplo.c
 *
 * @brief Exemplo de uso do driver ledc_buzzer_driver.
 *
 * Toca uma melodia com notas musicais e demonstra o modo pulse do buzzer.
 * Demonstra:
 *   - LEDBuzzerInit() para inicializar o periférico;
 *   - LEDBuzzerSet() para ligar/desligar com frequência dinâmica;
 *   - LEDBuzzerPulse() para pulsos PWM programáveis;
 *
 * @author Vinicius Silva <silva.viniciusr@gmail.com>
 *
 * @date 01 de Outubro de 2025
 */

#include <stdio.h>
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ledc_buzzer_driver.h"

static const char *s_TAG = "LEDC_BUZZER_EX";

/* Notas musicais em Hz (C4 a C5, oitava) */
static const uint32_t s_notes[] = {
    523,   /* C4 (Dó)   */
    587,   /* D4 (Ré)   */
    659,   /* E4 (Mi)   */
    698,   /* F4 (Fá)   */
    784,   /* G4 (Sol)  */
    880,   /* A4 (Lá)   */
    988,   /* B4 (Si)   */
    1046,  /* C5 (Dó')  */
};
#define NOTE_COUNT        (sizeof(s_notes) / sizeof(s_notes[0]))
#define NOTE_DUTY_MS      300     /* Duração de cada nota em ms */
#define PAUSE_BETWEEN_MS  100     /* Pausa entre melodias em ms */


void app_main(void) {
    esp_err_t rc;

    ESP_LOGI(s_TAG, "=== Exemplo: LEDC Buzzer Driver ===");
    ESP_LOGI(s_TAG, "Inicializando ...");

    rc = LEDBuzzerInit();
    if (rc != ESP_OK) {
        ESP_LOGE(s_TAG, "Falha ao inicializar o buzzer. rc: %d", rc);
        return;
    }

    /* ========================================================== *
     * Demonstração 1: notas musicais                               *
     * ========================================================== */
    ESP_LOGI(s_TAG, "--- Notas musicais ---");
    for (;;) {
        ESP_LOGI(s_TAG, "Tocando melodia (nota %d/%d) ...", NOTE_COUNT, NOTE_COUNT);
        for (int i = 0; i < NOTE_COUNT; ++i) {
            ESP_LOGI(s_TAG, "Nota %2d: %.0f Hz", i + 1, (float)s_notes[i]);
            LEDBuzzerSet(1, s_notes[i]);
            vTaskDelay(pdMS_TO_TICKS(NOTE_DUTY_MS));

            /* Pausa curta entre notas */
            LEDBuzzerSet(0, 0);
            vTaskDelay(pdMS_TO_TICKS(PAUSE_BETWEEN_MS));
        }

        ESP_LOGI(s_TAG, "Melodia completa. Aguardando ...");
        vTaskDelay(pdMS_TO_TICKS(500));

        /* Tocar melodia de volta */
        ESP_LOGI(s_TAG, "Tocando na ordem inversa ...");
        for (int i = NOTE_COUNT - 1; i >= 0; --i) {
            ESP_LOGI(s_TAG, "Nota %2d: %.0f Hz", i + 1, (float)s_notes[i]);
            LEDBuzzerSet(1, s_notes[i]);
            vTaskDelay(pdMS_TO_TICKS(NOTE_DUTY_MS));

            LEDBuzzerSet(0, 0);
            vTaskDelay(pdMS_TO_TICKS(PAUSE_BETWEEN_MS));
        }
        ESP_LOGI(s_TAG, "Melodia inversa completa.\n");
    }

    /* Nunca chega aqui */
}
