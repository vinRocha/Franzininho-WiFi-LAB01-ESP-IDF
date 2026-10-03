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
 * @file input_driver.c
 *
 * @brief Driver para leitura do estado dos botoes BT1-BT6 da placa Frazininho WiFi-LAB01.
 *
 * Tarefa e responsavel pela leitura dos estados.
 *
 * @author Vinicius Silva <silva.viniciusr@gmail.com>
 *
 * @date 01 de outubro de 2026
 *
 */

#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "input_driver.h"

static const char *s_TAG = "INPUT_D";

enum BUTTON_GPIOS
{
  BT1 = GPIO_NUM_7,
  BT2 = GPIO_NUM_6,
  BT3 = GPIO_NUM_5,
  BT4 = GPIO_NUM_4,
  BT5 = GPIO_NUM_3,
  BT6 = GPIO_NUM_2
};

#define INPUT_DELAY_MS   (30) //check buttons states at about 30x per second.
#define BUTTON_GPIO_PINS ( (1LLU << BT1) | (1LLU << BT2) | (1LLU << BT3) | \
                           (1LLU << BT4) | (1LLU << BT5) | (1LLU << BT6) )

struct driver_ctx {
/* Ultima leitura realizada */
  input_data_t last_read;
/* Variavel para verificar erro */
  esp_err_t rc;
};

/* Ponteiro global para acesso ao contexto do driver */
static struct driver_ctx *s_dctx_p = NULL;

/* Mutex para indicar se driver encontra-se ocupado
 * Uma vez inicializado, esse mutex nao pode ser desalocado
 */
static SemaphoreHandle_t s_driver_mutex = NULL;

/**
 * @brief Inicializacao privada do driver de input.
 *
 * inicializa parametros de s_dctx_p.
 *
 * @return
 *    - ESP_OK (0): Success
 *    - Negative value: Error
 *
 */
static esp_err_t s_InputInit(void);

/**
 * @brief Desinicializacao privada do driver de input.
 *
 * Deleta a tarefa principal do driver e limpa s_dctx_p.
 * Espera que s_driver_mutex esteja adquirido.
 *
 */
static void s_InputCleanup(void);

/**
 * @brief Realiza a leitura dos botoes
 *
 * Helper function para realizar a leitura dos 6 botoes
 * na estrutura input_data_t passada via ponteiro.
 *
 */
inline static void s_ReadButtons(input_data_t *s0);

/**
 * @brief Loop principal da tarefa INPUT_D
 *
 * Nao deve retornar.
 *
 * @param pvParameters ponteiro para dados.
 * Nao utilizado. arg = NULL
 *
 */
static void s_InputTask(void *pvParameters) {

  struct driver_ctx d_ctx = {0};

  if (!s_driver_mutex) {
    s_driver_mutex = xSemaphoreCreateMutex();
    if (!s_driver_mutex) vTaskDelete(NULL);
  }

  while(!xSemaphoreTake(s_driver_mutex, portMAX_DELAY));
  s_dctx_p = &d_ctx;

  if (s_InputInit()) {
    ESP_LOGE(s_TAG, "Erro durante a initializacao do driver.\n"
                    "error code: %d", d_ctx.rc);
    s_InputCleanup();
    return;
  }

  xSemaphoreGive(s_driver_mutex);

  for (;;) {
    while(!xSemaphoreTake(s_driver_mutex, portMAX_DELAY));

    input_data_t s0, s1;

    uint8_t *s0_p = (uint8_t*) &s0;
    uint8_t *s1_p = (uint8_t*) &s1;

    //Deboucing....
    do
    {
      s_ReadButtons(&s0);
      /* Delay minimo para tarefas do freeRTOS = 10ms -> Tick rate de 100Hz,
       * o que suficiente para um deboucing the push button operado por humanos. */
      vTaskDelay(pdMS_TO_TICKS(10));
      s_ReadButtons(&s1);
    } while (*s0_p != *s1_p);

    s_dctx_p->last_read = s0;
    xSemaphoreGive(s_driver_mutex);
    vTaskDelay(pdMS_TO_TICKS(INPUT_DELAY_MS));
  }

//Nao deveria chegar aqui...
  ESP_LOGE(s_TAG, "Driver error.");
  s_InputCleanup();
  return;
}

esp_err_t s_InputInit(void) {

  const gpio_config_t gpio_handle = {
    .pin_bit_mask = BUTTON_GPIO_PINS,
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
  };

  s_dctx_p->rc = gpio_config(&gpio_handle);
  return s_dctx_p->rc;
}

void s_InputCleanup(void) {

  s_dctx_p = NULL;
  xSemaphoreGive(s_driver_mutex);
  ESP_LOGE(s_TAG, "Deletando a tarefa %s...", s_TAG);
  vTaskDelete(NULL);
}

inline void s_ReadButtons(input_data_t *s0)
{
  s0->bt1 = gpio_get_level(BT1);
  s0->bt2 = gpio_get_level(BT2);
  s0->bt3 = gpio_get_level(BT3);
  s0->bt4 = gpio_get_level(BT4);
  s0->bt5 = gpio_get_level(BT5);
  s0->bt6 = gpio_get_level(BT6);
  return;
}

/* Init publico do driver de input */
esp_err_t InputInit() {

  if (s_dctx_p) {
    return ESP_ERR_NOT_ALLOWED;
  }

  /*  Registra a tarefa INPUT_D */
  if (xTaskCreate(s_InputTask, s_TAG, CONFIG_INPUT_TASK_STACK_SIZE, NULL,
                  CONFIG_INPUT_TASK_PRIORITY, NULL) != pdPASS) {
    ESP_LOGE(s_TAG, "Erro criando a tarefa %s...", s_TAG);
    return ESP_FAIL;
  }
  return ESP_OK;
}

esp_err_t InputRead(input_data_t *input_data) {

  if (!input_data)
    return ESP_ERR_INVALID_ARG;

  if (!s_driver_mutex)
    return ESP_ERR_INVALID_STATE;

  if (!xSemaphoreTake(s_driver_mutex, pdMS_TO_TICKS(INPUT_DELAY_MS)))
    return ESP_ERR_TIMEOUT;

  if (!s_dctx_p) {
    xSemaphoreGive(s_driver_mutex);
    return ESP_ERR_INVALID_STATE;
  }

  uint8_t *s0_p = (uint8_t*) input_data;
  uint8_t *s1_p = (uint8_t*) &s_dctx_p->last_read;

  s_dctx_p->last_read.changed = input_data->changed;
  if (*s0_p != *s1_p) s_dctx_p->last_read.changed = 1;
  else s_dctx_p->last_read.changed = 0;

  *input_data = s_dctx_p->last_read;
  xSemaphoreGive(s_driver_mutex);
  return ESP_OK;
}
