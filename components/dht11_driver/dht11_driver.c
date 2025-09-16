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
 * @file dht11_driver.c
 *
 * @brief Driver para o sensor DHT11.
 *
 * Tarefa e responsavel pela ativacao da leitura e composicao do resultado.
 * Comunicacao com o sensor via interrupcao de GPIO.
 *
 * @author Vinicius Silva <silva.viniciusr@gmail.com>
 *
 * @date 14 de Maio de 2025
 *
 */

#include <string.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_intr_alloc.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "dht11_driver.h"

#define DHT11_BYTES                     5
#define DHT11_BITS_EXPECTED             42
/* TIMEOUT_MS deveria ser de 5MS, mas 10MS e o menor intervalo de tempo que
 * uma tarefa pode bloquear devido ao freertos tick Hz = 100. Os intervalos
 * devem ser definidos em multiplos de 10MS.
 */
#define DHT11_TIMEOUT_MS                10
#define DHT11_TRIGGER_DELAY_MS          20

static const char *s_TAG = "DHT11_D";

/* Datasheet do sensor:
 * https://www.mouser.com/datasheet/2/758/DHT11-Technical-Data-Sheet-Translated-Version-1143054.pdf
 */

struct driver_ctx {
/* Contador de bits recebidos */
  int bit_count;

/* Bit stream recebido do sensor */
  uint64_t bitstream;

/* Bytes recebidos do sensor */
  unsigned char bytes[DHT11_BYTES];

/* Variaves para se calcular delta de tempo entre os bits recebidos */
  int64_t current_time;
  int64_t previous_time;

/* Handle da tarefa para que se possa disparar a leitura do sensor */
  TaskHandle_t task_handle;

/* Variavel para indicar se a leitura esta pronta */
  bool data_ready;

/* Variavel para verificar erro */
  esp_err_t rc;
};

static struct driver_ctx *s_d_ctxp = NULL;

/**
 * @brief Callback de interupcao do GPIO.
 *
 * Callback para ler intervalo de tempo associado ao
 * bit envidado pelo sensor DHT11.
 *
 * @param arg ponteiro para dados.
 * Nao utilizado. arg = NULL
 *
 */
static void s_GpioRecvData(void *arg);

/**
 * @brief Inicializacao interna do driver DHT11.
 *
 * Configura o pino GPIO do sensor e inicializa
 * parametros de s_d_ctxp.
 *
 * @return
 *    - ESP_OK (0): Success
 *    - Negative value: Error
 *
 */
static esp_err_t s_Dht11Init(void);

/**
 * @brief Loop principal da tarefa DHT11
 *
 * Nao deve retornar.
 *
 * @param pvParameters ponteiro para dados.
 * Nao utilizado. arg = NULL
 *
 */
static void s_Dht11Task(void *pvParameters) {

  struct driver_ctx d_ctx;
  s_d_ctxp = &d_ctx;

  if (s_Dht11Init()) {
    ESP_LOGE(s_TAG, "Erro durante a initializacao do driver.\n"
                    "error code: %d", d_ctx.rc);
    s_d_ctxp = NULL;
    vTaskDelete(NULL);
    return;
  }

  for (;;) {
    vTaskSuspend(NULL); //aguarda solicitacao de nova leitura.
    d_ctx.data_ready = false;

    //Configura CONFIG_DHT11_GPIO para zero por > 20ms para disparar uma nova leitura.
    if (gpio_set_level(CONFIG_DHT11_GPIO, 0)) {
      d_ctx.rc = -1; //gpio_error
      goto driver_error;
    }
    vTaskDelay(DHT11_TRIGGER_DELAY_MS / portTICK_PERIOD_MS);

    d_ctx.bit_count = 0;
    if (gpio_set_level(CONFIG_DHT11_GPIO, 1)) {
      d_ctx.rc = -1; //gpio_error
      goto driver_error;
    }

    for (uint32_t notification;;) {
      notification = ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(DHT11_TIMEOUT_MS));
      if (notification == 1) {
        d_ctx.bitstream <<= 1;
        if ((d_ctx.current_time - d_ctx.previous_time) > 100)
          d_ctx.bitstream++;
        d_ctx.previous_time = d_ctx.current_time;
        d_ctx.bit_count++;
        continue;
      }
      else if (notification > 1) {
        ESP_LOGE(s_TAG, "Erro durante a leitura do sensor... Tente novamente...\n"
                 "notification: %lu", notification);
      }
      break;
    }

    if (d_ctx.bit_count < DHT11_BITS_EXPECTED) { //erro de leitura
      ESP_LOGE(s_TAG, "d_ctx.bit_count: %d", d_ctx.bit_count);
      memset(d_ctx.bytes, 0, sizeof(d_ctx.bytes));
      vTaskDelay(1);
      ulTaskNotifyTake(pdTRUE, 0);
      d_ctx.data_ready = true;
      continue;
    }

    //Popula os bits recebidos em d_ctx.bytes[]
    for (int i = 0; i < DHT11_BYTES; i++) {
       d_ctx.bytes[i] = (d_ctx.bitstream >> (32 - 8 * i)) & 0xff;
    }

    if (d_ctx.bytes[4] != d_ctx.bytes[0] + d_ctx.bytes[1] +
                         d_ctx.bytes[2] + d_ctx.bytes[3])
      ESP_LOGW(s_TAG, "Warning! Checksum incorreto!");

    d_ctx.data_ready = true;
  }

//Nao deveria chegar aqui, a nao ser com erro.
driver_error:
  switch (d_ctx.rc) {
    case -1:
      ESP_LOGE(s_TAG, "Erro durante set do GPIO.");
      break;
    default:
      ESP_LOGE(s_TAG, "Driver error.");
  }

  ESP_LOGE(s_TAG, "Deletando a tarefa %s...", s_TAG);
  s_d_ctxp = NULL;
  vTaskDelete(NULL);

  return;
}

esp_err_t s_Dht11Init(void) {

  s_d_ctxp->rc = ESP_OK;
  memset(s_d_ctxp->bytes, 0, sizeof(s_d_ctxp->bytes));
  s_d_ctxp->previous_time = 0;
  s_d_ctxp->data_ready = true;

  //Para trigar nova medicao ou retomar execucao da tarefa apos medicao
  s_d_ctxp->task_handle = xTaskGetCurrentTaskHandle();

  const  gpio_config_t gpio_handle = {
    .pin_bit_mask = 1LLU << CONFIG_DHT11_GPIO,
    .mode = GPIO_MODE_INPUT_OUTPUT_OD,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_NEGEDGE
  };

  if ((s_d_ctxp->rc= gpio_config(&gpio_handle)))
    return s_d_ctxp->rc = -1;

  if((s_d_ctxp->rc = gpio_set_level(CONFIG_DHT11_GPIO, 1)))
    return s_d_ctxp->rc = -2;

/* TODO gpio_install_isr_service e gpio_isr_handler_add deveriam ficar
 * estar em uma outra biblioteca que tem como funcao gerenciar os registros
 * de interrupcoes de gpio.
 */

  if ((s_d_ctxp->rc = gpio_install_isr_service(ESP_INTR_FLAG_LOWMED|ESP_INTR_FLAG_EDGE)))
    return s_d_ctxp->rc = -3;

  if ((s_d_ctxp->rc = gpio_isr_handler_add(CONFIG_DHT11_GPIO, s_GpioRecvData, NULL)))
    return s_d_ctxp->rc = -4;

  return s_d_ctxp->rc;
}


void s_GpioRecvData(void *arg) {

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  s_d_ctxp->current_time = esp_timer_get_time();
  vTaskNotifyGiveFromISR(s_d_ctxp->task_handle, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* Init publico do driver DHT11 */
esp_err_t Dht11Init() {

  if (s_d_ctxp) {
    return ESP_ERR_NOT_ALLOWED;
  }

  /*  Registra a tarefa DHT11 */
  if (xTaskCreate(s_Dht11Task, s_TAG, CONFIG_DHT11_TASK_STACK_SIZE, NULL,
                  CONFIG_DHT11_TASK_PRIORITY, NULL) != pdPASS) {
    ESP_LOGE(s_TAG, "Erro criando a tarefa %s...", s_TAG);
    return ESP_FAIL;
  }
  return ESP_OK;
}

esp_err_t Dht11Update(void) {

  int64_t current_time;
  if (s_d_ctxp) {
    current_time = esp_timer_get_time();
    /* Pelo menos 1 segundo de intervalo entre as medicoes */
    if ((current_time - s_d_ctxp->previous_time) > 1000 * 1000) {
      vTaskResume(s_d_ctxp->task_handle);
    }
    return ESP_OK;
  }
  return ESP_ERR_INVALID_STATE;
}

esp_err_t Dht11Read(dht11_data_t *dht11_data) {

  if (!dht11_data) {
    return ESP_ERR_INVALID_ARG;
  }
  if (s_d_ctxp) {
    if (s_d_ctxp->data_ready) {
      //Converte ponto fixo em ponto flutuante...
      float value = s_d_ctxp->bytes[0];
      dht11_data->relative_humidity = value;
      int i = 10;
      while (s_d_ctxp->bytes[1] / i) {
        i = i * 10;
      }
      value = i;
      value = s_d_ctxp->bytes[1] / value;
      dht11_data->relative_humidity += value;

      value = s_d_ctxp->bytes[2];
      dht11_data->temperature = value;
      i = 10;
      while (s_d_ctxp->bytes[3] / i) {
        i = i * 10;
      }
      value = i;
      value = s_d_ctxp->bytes[3] / value;
      dht11_data->temperature += value;

      return ESP_OK;
    }
    return ESP_ERR_NOT_FINISHED;
  }
  return ESP_ERR_INVALID_STATE;
}

