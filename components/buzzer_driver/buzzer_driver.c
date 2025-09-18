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
 * @file buzzer_driver.c
 *
 * @brief Driver para buzzer.
 *
 * Tarefa e responsavel pela configuracao, ativacao e desativacao do buzzer.
 *
 * @author Vinicius Silva <silva.viniciusr@gmail.com>
 *
 * @date 29 de Maio de 2025
 */

#include "esp_log.h"
#include "driver/dac_cosine.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "buzzer_driver.h"

#ifdef CONFIG_BUZZER_ATTENUATION_DB_0
#define BUZZER_ATTENUATION DAC_COSINE_ATTEN_DB_0
#elifdef CONFIG_BUZZER_ATTENUATION_DB_6
#define BUZZER_ATTENUATION DAC_COSINE_ATTEN_DB_6
#elifdef CONFIG_BUZZER_ATTENUATION_DB_12
#define BUZZER_ATTENUATION DAC_COSINE_ATTEN_DB_12
#endif

static const char *s_TAG = "BUZZER_D";

struct driver_ctx {
/* Handle da tarefa para que se possa disparar modo pulse_mode */
  TaskHandle_t task_handle;

/* Periodo em MS para tocar o buzzer (BuzzerPulse) */
  unsigned period_on;

/* Periodo em MS para desligar o buzzer (BuzzerPulse) */
  unsigned period_off;

/* Handle para output de onda cossenoidal no GPIO_17 -> DAC_0 */
  dac_cosine_handle_t dac0_handle;

/* Variavel para verificar possivel erro */
  esp_err_t rc;
};
static struct driver_ctx *s_d_ctxp = NULL;

/**
 * @brief Inicializacao interna do driver do buzzer.
 *
 * Configura DAC cossenoidal no IO do buzzer.
 *
 * @return
 *    - ESP_OK (0): Success
 *    - Negative value: Error
 *
 */
static esp_err_t s_BuzzerInit(void);

/**
 * @brief Loop principal da tarefa BUZZER_D
 *
 * Nao deve retornar. Implementa o modo pulse.
 *
 * @param pvParameters ponteiro para dados.
 * Nao utilizado. arg = NULL
 *
 */
static void s_BuzzerTask(void *pvParameters) {

  struct driver_ctx d_ctx;
  s_d_ctxp = &d_ctx;

  if (s_BuzzerInit()) {
    ESP_LOGE(s_TAG, "Erro durante a initializacao do driver.\n"
                    "error code: %d", d_ctx.rc);
    s_d_ctxp = NULL;
    vTaskDelete(NULL);
    return;
  }
  vTaskSuspend(NULL);

  /* Loop principal */
  /* Implementa buzzer pulse_mode */
  for (;;) {
    //Ativa DAC cossenoidal por period_on ms.
    dac_cosine_start(d_ctx.dac0_handle);
    vTaskDelay((TickType_t) d_ctx.period_on / portTICK_PERIOD_MS);

    //Desliga dac cossenoidal por period_off ms.
    dac_cosine_stop(d_ctx.dac0_handle);
    vTaskDelay((TickType_t) d_ctx.period_off / portTICK_PERIOD_MS);
  }

  //Nao deveria chegar aqui...
  dac_cosine_del_channel(d_ctx.dac0_handle);
  s_d_ctxp = NULL;
  return;
}

esp_err_t s_BuzzerInit(void) {

  s_d_ctxp->rc = ESP_OK;

  //Para suspender ou retomar loop principal
  s_d_ctxp->task_handle = xTaskGetCurrentTaskHandle();

  dac_cosine_config_t cos0_cfg = {
        .chan_id = DAC_CHAN_0,
        .freq_hz = CONFIG_BUZZER_FREQUENCY,
        .clk_src = DAC_COSINE_CLK_SRC_DEFAULT,
        .offset = 0,
        .phase = DAC_COSINE_PHASE_0,
        .atten = BUZZER_ATTENUATION,
        .flags.force_set_freq = true,
  };
  s_d_ctxp->rc = dac_cosine_new_channel(&cos0_cfg, &(s_d_ctxp->dac0_handle));
  return s_d_ctxp->rc;
}


/* Init publico do driver do buzzer.
 * Apenas registra a tarefa no sistema, a inicializacao
 * do dac_0 e realizada no init privado. */
esp_err_t BuzzerInit(TaskHandle_t *task) {

  if (s_d_ctxp)
    return ESP_ERR_NOT_ALLOWED;

  /*  Registra a tarefa BUZZER_D */
  if (xTaskCreate(s_BuzzerTask, s_TAG, CONFIG_BUZZER_TASK_STACK_SIZE, NULL,
                  CONFIG_BUZZER_TASK_PRIORITY, task) != pdPASS) {
    ESP_LOGE(s_TAG, "Erro criando a tarefa %s...", s_TAG);
    return ESP_FAIL;
  }
  return ESP_OK;
}

esp_err_t BuzzerSet(char value) {

  if (!s_d_ctxp)
    return ESP_ERR_INVALID_STATE;

  vTaskSuspend(s_d_ctxp->task_handle);
  if (value)
    dac_cosine_start(s_d_ctxp->dac0_handle);
  else
    dac_cosine_stop(s_d_ctxp->dac0_handle);
  return ESP_OK;
}

esp_err_t BuzzerPulse(unsigned period, unsigned duty_cycle) {

  if (!s_d_ctxp)
    return ESP_ERR_INVALID_STATE;

  duty_cycle = duty_cycle > 100 ? 100 : duty_cycle;
  if (period) {
    s_d_ctxp->period_on = period * duty_cycle / 100;
    s_d_ctxp->period_off = period - s_d_ctxp->period_on;
    vTaskResume(s_d_ctxp->task_handle);
  } else {
    vTaskSuspend(s_d_ctxp->task_handle);
    dac_cosine_stop(s_d_ctxp->dac0_handle);
  }
  return ESP_OK;
}
