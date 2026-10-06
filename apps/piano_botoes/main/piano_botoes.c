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
 * @file piano_botoes.c
 *
 * @brief Piano de botoes: cada botao da Franzininho WiFi LAB01 toca uma nota
 * musical no buzzer enquanto permanece pressionado.
 *
 * A aplicacao associa os seis botoes (BT1-BT6) as notas da escala de Do maior
 * (Do, Re, Mi, Fa, Sol, La - uma oitava a partir de C4). O estado dos botoes e
 * lido periodicamente pelo driver de input, que ja realiza o debouncing via
 * software. A cada mudanca de estado a aplicacao aciona o driver LEDC do buzzer:
 *
 *   - botao pressionado (transicao solta -> pressionada): toca a nota associada;
 *   - botao solto (transicao pressionada -> solta): SILENCIA o buzzer, ou, caso
 *     outro botao ainda esteja pressionado, toca a nota dele;
 *   - nenhum botao pressionado: buzzer desligado.
 *
 * Caso mais de um botao seja pressionado ao mesmo tempo, a nota do ultimo botao
 * pressionado tem prioridade; ao solta-lo a aplicacao retorna para uma das notas
 * ainda pressionadas (a de menor indice).
 *
 * @author Vinicius Silva <silva.viniciusr@gmail.com>
 *
 * @date 06 de outubro de 2026
 */

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "input_driver.h"
#include "ledc_buzzer_driver.h"

static const char *s_TAG = "app_main";

/* Quantidade de botoes/notas suportadas */
#define NOTE_COUNT (6)

/* Periodo de varredura dos botoes em milissegundos.
 * O driver de input ja realiza o debouncing, mas mantemos uma taxa de
 * varredura confortavel para resposta tatil ao usuario. */
#define POLL_PERIOD_MS (20)

/* Descricao de uma nota musical associada a um botao */
typedef struct
{
  const char *name;  /* Nome da nota (para o log) */
  int freq_hz;       /* Frequencia da nota em Hertz */
} note_t;

/* Mapa de notas: indice 0 -> BT1, indice 1 -> BT2, ... indice 5 -> BT6.
 * Escala de Do maior (uma oitava a partir de C4). */
static const note_t s_notes[NOTE_COUNT] = {
    {"Do",  262},  /* BT1 - C4  */
    {"Re",  294},  /* BT2 - D4  */
    {"Mi",  330},  /* BT3 - E4  */
    {"Fa",  349},  /* BT4 - F4  */
    {"Sol", 392},  /* BT5 - G4  */
    {"La",  440},  /* BT6 - A4  */
};

/**
 * @brief Converte a estrutura de leitura do driver de input em uma mascara de bits.
 *
 * Os botoes sao ativos em nivel baixo (pressed = 0), portanto invertemos os
 * bits para que 1 represente "pressionado".
 *
 * @param input_data Leitura atual dos botoes.
 *
 * @return Mascara com um bit por botao (bit 0 = BT1 ... bit 5 = BT6).
 */
static uint8_t s_PressedMask(const input_data_t *input_data)
{
  return (uint8_t) ((!input_data->bt1 << 0) |
                    (!input_data->bt2 << 1) |
                    (!input_data->bt3 << 2) |
                    (!input_data->bt4 << 3) |
                    (!input_data->bt5 << 4) |
                    (!input_data->bt6 << 5));
}

/**
 * @brief Retorna o indice do botao pressionado de menor indice na mascara.
 *
 * @param mask Mascara de botoes pressionados (bit 0 = BT1 ... bit 5 = BT6).
 *
 * @return Indice do botao (0-5) ou -1 se a mascara for zero.
 */
static int s_FirstPressed(uint8_t mask)
{
  for (int i = 0; i < NOTE_COUNT; ++i)
  {
    if (mask & (1U << i))
      return i;
  }
  return -1;
}

/**
 * @brief Inicializa os drivers necessarios para a aplicacao.
 *
 * @return
 *    - ESP_OK (0): Success.
 *    - ESP_ERR_INVALID_STATE: Falha ao inicializar algum driver.
 */
static esp_err_t s_InitDrivers(void)
{
  /* Inicia o driver de input (botoes) */
  if (InputInit() != ESP_OK)
  {
    ESP_LOGE(s_TAG, "Erro ao inicializar o driver de input...\n");
    return ESP_ERR_INVALID_STATE;
  }

  /* Inicia o driver LEDC do buzzer */
  if (LedcBuzzerInit() != ESP_OK)
  {
    ESP_LOGE(s_TAG, "Erro ao inicializar o driver do buzzer...\n");
    return ESP_ERR_INVALID_STATE;
  }

  return ESP_OK;
}

/**
 * @brief Loop principal
 *
 * Pode retornar em caso de erro.
 *
 */
void app_main(void)
{
  /* Ultima mascara de botoes pressionados observada (edge detection) */
  uint8_t prev_mask = 0;
  /* Nota atualmente tocando (-1 = buzzer silencioso) */
  int active = -1;

  /* Aguarda 2 segundos para conclusao da inicializacao do HW (USB_CDC) */
  vTaskDelay(pdMS_TO_TICKS(2000));

  ESP_LOGI(s_TAG, "=== Piano de Botoes (Franzininho WiFi LAB01) ===");
  ESP_LOGI(s_TAG, "Mapa de notas:");
  for (int i = 0; i < NOTE_COUNT; ++i)
  {
    ESP_LOGI(s_TAG, "  BT%d -> %-3s (%d Hz)", i + 1, s_notes[i].name,
             s_notes[i].freq_hz);
  }

  /* Inicia os drivers da aplicacao */
  if (s_InitDrivers() != ESP_OK)
    return;

  /* Loop infinito da aplicacao */
  for (;;)
  {
    input_data_t input_data;

    if (InputRead(&input_data) == ESP_OK)
    {
      uint8_t mask = s_PressedMask(&input_data);
      /* Botoes que passaram de solto para pressionado nesta varredura */
      uint8_t newly_pressed = mask & ~prev_mask;

      if (newly_pressed)
      {
        /* Ultimo botao pressionado assume a nota */
        active = s_FirstPressed(newly_pressed);
        LedcBuzzerSet(1, s_notes[active].freq_hz);
        ESP_LOGI(s_TAG, "BT%d -> %s (%d Hz)", active + 1, s_notes[active].name,
                 s_notes[active].freq_hz);
      }
      else if (active >= 0 && !(mask & (1U << active)))
      {
        /* A nota ativa foi solta: volta para outro botao ainda pressionado
         * ou silencia o buzzer caso nenhum reste. */
        active = s_FirstPressed(mask);
        if (active >= 0)
        {
          LedcBuzzerSet(1, s_notes[active].freq_hz);
          ESP_LOGI(s_TAG, "BT%d -> %s (%d Hz)", active + 1, s_notes[active].name,
                   s_notes[active].freq_hz);
        }
        else
        {
          LedcBuzzerSet(0, 0);
          ESP_LOGI(s_TAG, "Buzzer OFF");
        }
      }

      prev_mask = mask;
    }

    vTaskDelay(pdMS_TO_TICKS(POLL_PERIOD_MS));
  }

  /* Nao deve chegar aqui!! */
  return;
}