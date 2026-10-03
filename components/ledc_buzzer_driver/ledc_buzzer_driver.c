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
 * @file ledc_buzzer_driver.c
 *
 * @brief Driver para buzzer via periférico LEDC (PWM).
 *
 * Tarefa e responsavel pela configuracao, ativacao e desativacao do buzzer
 * usando o periférico LEDC da familia ESP32-S2. Toda a interacao com o
 * periférico ocorre no contexto da tarefa LEDC_BUZZER_D, de modo que as
 * funcoes publicas nao bloqueiam a aplicacao.
 *
 * @author Vinicius Silva <silva.viniciusr@gmail.com>
 *
 * @date 03 de outubro de 2026
 */

#include "esp_log.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "ledc_buzzer_driver.h"

#define BUZZER_GPIO            CONFIG_LEDC_BUZZER_GPIO
#define LEDC_TIMER             LEDC_TIMER_0
#define LEDC_CHANNEL           LEDC_CHANNEL_0
#define LEDC_SPEED_MODE        LEDC_LOW_SPEED_MODE
/* Resolucao de duty de 6 bits (64 niveis).
 * Com REF_TICK (1 MHz) o periferico LEDC aceita f de ~16 Hz a 15625 Hz
 * (f = REF_TICK / (2^res * div), com div entre 1 e ~1024). */
#define LEDC_DUTY_RES          LEDC_TIMER_6_BIT
/* Valor maximo de duty para a resolucao configurada (2^res = 64).
 * O IDF considera o range de duty como [0, 2^res]. */
#define LEDC_DUTY_MAX          (1U << LEDC_DUTY_RES)
/* Duty para tom continuo (~50% -> 32/64) */
#define LEDC_DUTY_50           (1U << (LEDC_DUTY_RES - 1))
/* Limite inferior suportado pelo LEDC nesta resolucao (40 Hz) */
#define LEDC_FREQ_MIN          (40)
/* Limite superior do LEDC nesta resolucao (REF_TICK / 2^6 = 15625 Hz) */
#define LEDC_FREQ_MAX          (15625)
#define LEDC_BUZZER_TIMEOUT_MS (100)

static const char *s_TAG = "LEDC_BUZZER_D";

/* Comandos processados pela tarefa principal do driver */
enum ledc_buzzer_cmd
{
  CMD_NONE = 0,  /* sem comando pendente (tarefa dormindo) */
  CMD_SET,       /* aplica tom continuo (on/off) */
  CMD_PULSE      /* ativa modo pulse periodico */
};

struct driver_ctx
{
/* Handle da tarefa para que se possa disparar modo pulse */
  TaskHandle_t task_handle;
/* Ultimo comando solicitado pela aplicacao */
  enum ledc_buzzer_cmd cmd;
/* Estado ON/OFF para o comando CMD_SET */
  uint8_t on;
/* Frequencia solicitada para o comando CMD_SET */
  int freq_hz;
/* Duty (em bits do resolvedor) usado no modo pulse */
  uint32_t pulse_duty;
/* Periodo em MS que o buzzer fica ligado (LedcBuzzerPulse) */
  unsigned period_on;
/* Periodo em MS que o buzzer fica desligado (LedcBuzzerPulse) */
  unsigned period_off;
/* Variavel para verificar possivel erro */
  esp_err_t rc;
};

/* Ponteiro global para acesso ao contexto do driver */
static struct driver_ctx *s_d_ctx_p = NULL;

/* Mutex para indicar se driver encontra-se ocupado
 * Uma vez inicializado, esse mutex nao pode ser desalocado
 */
static SemaphoreHandle_t s_driver_mutex = NULL;

/**
 * @brief Inicializacao privada do driver LEDC do buzzer.
 *
 * Configura o timer e o canal LEDC do buzzer e inicializa
 * parametros de s_d_ctx_p.
 *
 * @return
 *    - ESP_OK (0): Success
 *    - Negative value: Error
 *
 */
static esp_err_t s_LedcBuzzerInit(void);

/**
 * @brief Desinicializacao privada do driver LEDC do buzzer.
 *
 * Deleta a tarefa principal do driver e limpa s_d_ctx_p.
 * Espera que s_driver_mutex esteja adquirido.
 *
 */
static void s_LedcBuzzerCleanup(void);

/**
 * @brief Atualiza o duty cycle do canal LEDC do buzzer.
 *
 * @param duty Duty em bits do resolvedor (0 - LEDC_DUTY_MAX).
 *
 */
static void s_LedcBuzzerSetDuty(uint32_t duty);

/**
 * @brief Loop principal da tarefa LEDC_BUZZER_D
 *
 * Nao deve retornar. A tarefa e responsavel por toda a interacao com o
 * periferico LEDC: processa os comandos enviados pelas funcoes publicas
 * e implementa o modo pulse.
 *
 * @param pvParameters ponteiro para dados.
 * Nao utilizado. arg = NULL
 *
 */
static void s_LedcBuzzerTask(void *pvParameters)
{
  struct driver_ctx d_ctx = {0};

  if (!s_driver_mutex)
  {
    s_driver_mutex = xSemaphoreCreateMutex();
    if (!s_driver_mutex) vTaskDelete(NULL);
  }

  while(!xSemaphoreTake(s_driver_mutex, portMAX_DELAY));
  s_d_ctx_p = &d_ctx;

  if (s_LedcBuzzerInit())
  {
    ESP_LOGE(s_TAG, "Erro durante a initializacao do driver.\n"
                    "error code: %d", d_ctx.rc);
    s_LedcBuzzerCleanup();
    return;
  }

  /* O mutex permanece adquirido; o loop abaixo o libera antes de dormir */

  for (;;)
  {
    /* Aguarda um comando, mantendo o mutex liberado durante o suspend */
    while (s_d_ctx_p->cmd == CMD_NONE)
    {
      xSemaphoreGive(s_driver_mutex);
      vTaskSuspend(NULL);
      while(!xSemaphoreTake(s_driver_mutex, portMAX_DELAY));
    }

    if (s_d_ctx_p->cmd == CMD_PULSE)
    {
      /* Modo pulse: repete ON/OFF ate receber um novo comando */
      while (s_d_ctx_p->cmd == CMD_PULSE)
      {
        s_LedcBuzzerSetDuty(s_d_ctx_p->pulse_duty);
        xSemaphoreGive(s_driver_mutex);
        vTaskDelay(pdMS_TO_TICKS(s_d_ctx_p->period_on));
        while(!xSemaphoreTake(s_driver_mutex, portMAX_DELAY));

        if (s_d_ctx_p->cmd != CMD_PULSE) break;

        s_LedcBuzzerSetDuty(0);
        xSemaphoreGive(s_driver_mutex);
        vTaskDelay(pdMS_TO_TICKS(s_d_ctx_p->period_off));
        while(!xSemaphoreTake(s_driver_mutex, portMAX_DELAY));
      }
      s_LedcBuzzerSetDuty(0);
    }
    else if (s_d_ctx_p->on)
    {
      /* Tom continuo: aplica uma vez, o HW mantem a onda */
      if ((s_d_ctx_p->rc = ledc_set_freq(LEDC_SPEED_MODE, LEDC_TIMER,
                                         s_d_ctx_p->freq_hz)) != ESP_OK)
      {
        ESP_LOGE(s_TAG, "Erro ao configurar frequencia (%d Hz). rc: %d",
                 s_d_ctx_p->freq_hz, s_d_ctx_p->rc);
      }
      s_LedcBuzzerSetDuty(LEDC_DUTY_50);
      s_d_ctx_p->cmd = CMD_NONE;
    }
    else
    {
      s_LedcBuzzerSetDuty(0);
      s_d_ctx_p->cmd = CMD_NONE;
    }
  }

//Nao deveria chegar aqui...
  ESP_LOGE(s_TAG, "Driver error.");
  s_LedcBuzzerCleanup();
  return;
}

static esp_err_t s_LedcBuzzerInit(void)
{
  ledc_timer_config_t timer_cfg = {0};
  ledc_channel_config_t ch_cfg = {0};

  s_d_ctx_p->rc = ESP_OK;
  s_d_ctx_p->task_handle = xTaskGetCurrentTaskHandle();
  s_d_ctx_p->cmd = CMD_NONE;
  s_d_ctx_p->freq_hz = CONFIG_LEDC_BUZZER_FREQ_DEFAULT;

  /* Configurar o timer do LEDC */
  timer_cfg.speed_mode      = LEDC_SPEED_MODE;
  timer_cfg.timer_num       = LEDC_TIMER;
  timer_cfg.duty_resolution = LEDC_DUTY_RES;
  timer_cfg.freq_hz         = CONFIG_LEDC_BUZZER_FREQ_DEFAULT;
  timer_cfg.clk_cfg         = LEDC_USE_REF_TICK;

#ifdef CONFIG_IDF_TARGET_ESP32S3
  timer_cfg.clk_cfg = LEDC_AUTO_CLK;
#endif

  if ((s_d_ctx_p->rc = ledc_timer_config(&timer_cfg)))
    return s_d_ctx_p->rc;

  /* Configurar o canal do LEDC */
  ch_cfg.speed_mode = LEDC_SPEED_MODE;
  ch_cfg.channel    = LEDC_CHANNEL;
  ch_cfg.timer_sel  = LEDC_TIMER;
  ch_cfg.intr_type  = LEDC_INTR_DISABLE;
  ch_cfg.gpio_num   = BUZZER_GPIO;
  ch_cfg.duty       = 0;

  if ((s_d_ctx_p->rc = ledc_channel_config(&ch_cfg)))
    return s_d_ctx_p->rc;

  ESP_LOGI(s_TAG, "Driver inicializado (GPIO %d, freq %d Hz).",
           BUZZER_GPIO, CONFIG_LEDC_BUZZER_FREQ_DEFAULT);
  return ESP_OK;
}

static void s_LedcBuzzerCleanup(void)
{
  s_d_ctx_p = NULL;
  xSemaphoreGive(s_driver_mutex);
  ESP_LOGE(s_TAG, "Deletando a tarefa %s...", s_TAG);
  vTaskDelete(NULL);
}

static void s_LedcBuzzerSetDuty(uint32_t duty)
{
  ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL, duty);
  ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL);
}

/* Init publico do driver LEDC do buzzer.
 * Apenas registra a tarefa no sistema, a inicializacao
 * do periferico LEDC e realizada no init privado. */
esp_err_t LedcBuzzerInit(void)
{
  if (s_d_ctx_p)
    return ESP_ERR_NOT_ALLOWED;

  /*  Registra a tarefa LEDC_BUZZER_D */
  if (xTaskCreate(s_LedcBuzzerTask, s_TAG, CONFIG_LEDC_BUZZER_TASK_STACK_SIZE, NULL,
                  CONFIG_LEDC_BUZZER_TASK_PRIORITY, NULL) != pdPASS)
  {
    ESP_LOGE(s_TAG, "Erro criando a tarefa %s...", s_TAG);
    return ESP_FAIL;
  }
  return ESP_OK;
}

esp_err_t LedcBuzzerSet(char value, int freq)
{
  if (!s_driver_mutex)
    return ESP_ERR_INVALID_STATE;

  /* Validar intervalo de frequencia (LEDC com 6 bits: 40 Hz - 15625 Hz).
   * Só valida quando value > 0 (ligar). */
  if (value > 0 && (freq < LEDC_FREQ_MIN || freq > LEDC_FREQ_MAX))
  {
    ESP_LOGE(s_TAG, "Frequencia invalida: %d Hz. Range: %d-%d.",
             freq, LEDC_FREQ_MIN, LEDC_FREQ_MAX);
    return ESP_ERR_INVALID_ARG;
  }

  if (!xSemaphoreTake(s_driver_mutex, pdMS_TO_TICKS(LEDC_BUZZER_TIMEOUT_MS)))
    return ESP_ERR_TIMEOUT;

  if (!s_d_ctx_p)
  {
    xSemaphoreGive(s_driver_mutex);
    return ESP_ERR_INVALID_STATE;
  }

  s_d_ctx_p->cmd     = CMD_SET;
  s_d_ctx_p->on      = value > 0 ? 1 : 0;
  s_d_ctx_p->freq_hz = freq;
  vTaskResume(s_d_ctx_p->task_handle);

  xSemaphoreGive(s_driver_mutex);
  return ESP_OK;
}

esp_err_t LedcBuzzerPulse(unsigned period, unsigned duty_cycle)
{
  if (!s_driver_mutex)
    return ESP_ERR_INVALID_STATE;

  if (duty_cycle > 100) duty_cycle = 100;

  if (!xSemaphoreTake(s_driver_mutex, pdMS_TO_TICKS(LEDC_BUZZER_TIMEOUT_MS)))
    return ESP_ERR_TIMEOUT;

  if (!s_d_ctx_p)
  {
    xSemaphoreGive(s_driver_mutex);
    return ESP_ERR_INVALID_STATE;
  }

  if (period)
  {
    s_d_ctx_p->cmd        = CMD_PULSE;
    s_d_ctx_p->pulse_duty = (uint32_t) duty_cycle * LEDC_DUTY_MAX / 100;
    s_d_ctx_p->period_on  = period * duty_cycle / 100;
    s_d_ctx_p->period_off = period - s_d_ctx_p->period_on;
  }
  else
  {
    /* Desliga o modo pulse e o buzzer */
    s_d_ctx_p->cmd     = CMD_SET;
    s_d_ctx_p->on      = 0;
    s_d_ctx_p->freq_hz = 0;
  }
  vTaskResume(s_d_ctx_p->task_handle);

  xSemaphoreGive(s_driver_mutex);
  return ESP_OK;
}