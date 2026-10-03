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
 * @file ldr_driver.c
 *
 * @brief Driver para o sensor LDR.
 *
 * Tarefa e responsavel pela configuracao e leitura do sensor LDR.
 *
 * @author Vinicius Silva <silva.viniciusr@gmail.com>
 *
 * @date 02 de outubro de 2026
 *
 */

#include <stdint.h>
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "ldr_driver.h"

static const char *s_TAG = "LDR_D";

#define LDR_TIMEOUT_MS (200)

struct driver_ctx
{
  adc_oneshot_unit_handle_t adc1_handle;
  adc_cali_handle_t adc1_chan0_cali_handle;
  int adc_raw;
  int voltage;
  uint8_t calibrated;
/* Handle da tarefa para que se possa disparar a leitura do sensor */
  TaskHandle_t task_handle;
/* Variavel para verificar erro */
  esp_err_t rc;
};

/* Ponteiro global para acesso ao contexto do driver */
static struct driver_ctx *s_dctx_p = NULL;

/* Mutex para indicar se driver encontra-se ocupado
 * Uma vez inicializado, esse mutex nao pode ser desalocado
 */
static SemaphoreHandle_t s_driver_mutex = NULL;

static uint8_t adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void adc_calibration_deinit(adc_cali_handle_t handle);

/**
 * @brief Inicializacao privada do driver LDR.
 *
 * Cria o mutex, configura o ADC para do sensor e inicializa s_dctx_p.
 *
 * @return
 *    - ESP_OK (0): Success
 *    - Negative value: Error
 *
 */
static esp_err_t s_LdrInit(void);

/**
 * @brief Desinicializacao privada do driver LDR.
 *
 * Limpa s_dctx_p e deleta a tarefa principal do driver.
 * Espera que s_driver_mutex esteja adquirido.
 *
 */
static void s_LdrCleanup(void);

/**
 * @brief Loop principal da tarefa LDR
 *
 * Nao deve retornar.
 *
 * @param pvParameters ponteiro para dados.
 * Nao utilizado. arg = NULL
 *
 */
static void s_LdrTask(void *pvParameters)
{
  struct driver_ctx d_ctx = {0};

  if (!s_driver_mutex)
  {
    s_driver_mutex = xSemaphoreCreateMutex();
    if (!s_driver_mutex) vTaskDelete(NULL);
  }

  while(!xSemaphoreTake(s_driver_mutex, portMAX_DELAY));
  s_dctx_p = &d_ctx;

  if (s_LdrInit())
  {
    ESP_LOGE(s_TAG, "Erro durante a initializacao do driver.\n"
                    "error code: %d", d_ctx.rc);
    s_LdrCleanup();
    return;
  }

  xSemaphoreGive(s_driver_mutex);

  for (;;)
  {
    xSemaphoreGive(s_driver_mutex);
    vTaskSuspend(NULL);
    while(!xSemaphoreTake(s_driver_mutex, portMAX_DELAY));
    if ((s_dctx_p->rc = adc_oneshot_read(s_dctx_p->adc1_handle, ADC_CHANNEL_0,
         &s_dctx_p->adc_raw)))
    {
      ESP_LOGE(s_TAG, "Erro ao ler o sensor LDR...");
      s_dctx_p->adc_raw = 0;
      s_dctx_p->voltage = 0;
    }
    if (s_dctx_p->calibrated)
    {
      if ((s_dctx_p->rc = adc_cali_raw_to_voltage(s_dctx_p->adc1_chan0_cali_handle,
           s_dctx_p->adc_raw, &s_dctx_p->voltage)))
      {
        ESP_LOGE(s_TAG, "Erro ao calibrar a leitura raw do sensor LDR...");
        s_dctx_p->adc_raw = 0;
        s_dctx_p->voltage = 0;
      }
    }
  }

//Nao deveria chegar aqui...
  ESP_LOGE(s_TAG, "Driver error.");
  s_LdrCleanup();
  return;
}

esp_err_t s_LdrInit(void)
{
  s_dctx_p->task_handle = xTaskGetCurrentTaskHandle();
  if (!s_dctx_p->task_handle)
    return -1;

  adc_oneshot_unit_init_cfg_t init_config1 = {
    .unit_id = ADC_UNIT_1
  };

  if ((s_dctx_p->rc = adc_oneshot_new_unit(&init_config1, &s_dctx_p->adc1_handle)))
    return s_dctx_p->rc;

  adc_oneshot_chan_cfg_t config = {
    .bitwidth = ADC_BITWIDTH_DEFAULT,
    .atten = ADC_ATTEN_DB_12
  };

  if ((s_dctx_p->rc = adc_oneshot_config_channel(s_dctx_p->adc1_handle,
                                                  ADC_CHANNEL_0, &config)))
    return s_dctx_p->rc;

  s_dctx_p->calibrated = adc_calibration_init(ADC_UNIT_1, ADC_CHANNEL_0, ADC_ATTEN_DB_12,
                                               &s_dctx_p->adc1_chan0_cali_handle);
  return s_dctx_p->rc;
}

void s_LdrCleanup(void)
{
  adc_calibration_deinit(s_dctx_p->adc1_chan0_cali_handle);
  s_dctx_p = NULL;
  xSemaphoreGive(s_driver_mutex);
  ESP_LOGE(s_TAG, "Deletando a tarefa %s...", s_TAG);
  vTaskDelete(NULL);
}

/* Init publico do driver LDR */
esp_err_t LdrInit()
{
  if (s_dctx_p)
    return ESP_ERR_NOT_ALLOWED;
  /*  Registra a tarefa LDR_D */
  if (xTaskCreate(s_LdrTask, s_TAG, CONFIG_LDR_TASK_STACK_SIZE, NULL,
                           CONFIG_LDR_TASK_PRIORITY, NULL) != pdPASS)
  {
    ESP_LOGE(s_TAG, "Erro criando a tarefa %s...", s_TAG);
    return ESP_FAIL;
  }
  return ESP_OK;
}

esp_err_t LdrRead(int *voltage)
{
  if (!voltage)
    return ESP_ERR_INVALID_ARG;

  if (!s_driver_mutex)
    return ESP_ERR_INVALID_STATE;

  if (!xSemaphoreTake(s_driver_mutex, pdMS_TO_TICKS(LDR_TIMEOUT_MS)))
    return ESP_ERR_TIMEOUT;

  if (!s_dctx_p)
  {
    xSemaphoreGive(s_driver_mutex);
    return ESP_ERR_INVALID_STATE;
  }
  /*
   * Realiza a leitura do sensor e salva na variavel voltage
   * recebida do usuario
   * */

  vTaskResume(s_dctx_p->task_handle);

  xSemaphoreGive(s_driver_mutex);
  vTaskDelay(pdMS_TO_TICKS(10));

  xSemaphoreTake(s_driver_mutex, portMAX_DELAY);
  if (s_dctx_p->calibrated)
    *voltage = s_dctx_p->voltage;
  else
    *voltage = s_dctx_p->adc_raw;
  xSemaphoreGive(s_driver_mutex);
  return ESP_OK;
}

/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
uint8_t adc_calibration_init(adc_unit_t unit, adc_channel_t channel,
                             adc_atten_t atten, adc_cali_handle_t *out_handle)
{

  adc_cali_handle_t handle = NULL;
  esp_err_t ret = ESP_FAIL;
  uint8_t calibrated = 0;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  if (!calibrated)
  {
    ESP_LOGI(s_TAG, "calibration scheme version is %s", "Curve Fitting");
    adc_cali_curve_fitting_config_t cali_config = {
      .unit_id = unit,
      .chan = channel,
      .atten = atten,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
    if (ret == ESP_OK) {
      calibrated = 1;
    }
  }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
  if (!calibrated)
  {
    ESP_LOGI(s_TAG, "calibration scheme version is %s", "Line Fitting");
    adc_cali_line_fitting_config_t cali_config = {
      .unit_id = unit,
      .atten = atten,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
    if (ret == ESP_OK)
      calibrated = 1;
  }
#endif

  *out_handle = handle;
  if (ret == ESP_OK)
    ESP_LOGI(s_TAG, "Calibration Success");
  else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated)
    ESP_LOGW(s_TAG, "eFuse not burnt, skip software calibration");
  else
    ESP_LOGE(s_TAG, "Invalid arg or no memory");

  return calibrated;
}

static void adc_calibration_deinit(adc_cali_handle_t handle)
{

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  ESP_LOGI(s_TAG, "deregister %s calibration scheme", "Curve Fitting");
  adc_cali_delete_scheme_curve_fitting(handle);

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
  ESP_LOGI(s_TAG, "deregister %s calibration scheme", "Line Fitting");
  adc_cali_delete_scheme_line_fitting(handle);
#endif
}
