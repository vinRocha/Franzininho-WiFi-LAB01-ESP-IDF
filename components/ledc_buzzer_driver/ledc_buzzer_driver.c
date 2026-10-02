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
 * @file ledc_buzzer_driver.c
 *
 * @brief Driver para buzzer via periférico LEDC (PWM).
 *
 * Tarefa e responsavel pela configuracao, ativacao e desativacao do buzzer
 * usando o periférico LEDC da famiglia ESP32-S2.
 *
 * @author Vinicius Silva <silva.viniciusr@gmail.com>
 *
 * @date 01 de Outubro de 2025
 */

#include "esp_log.h"
#include "esp_err.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ledc_buzzer_driver.h"

#define BUZZER_GPIO       17
#define LEDC_TIMER        LEDC_TIMER_0
#define LEDC_CHANNEL      LEDC_CHANNEL_0
#define LEDC_SPEED_MODE   LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES     LEDC_TIMER_8_BIT

static const char *s_TAG = "LEDC_BUZZER_D";

struct driver_ctx {
    int32_t   freq_hz;    /* Frequencia configurada ativamente */
    uint32_t  current_dut;/* Duty atual em bits do resolvedor */
    bool      initialized;
};

static struct driver_ctx s_dctx = {0};

static void s_LedcBuzzerSetDuty(uint32_t duty);

esp_err_t LEDBuzzerInit(void) {
    ledc_timer_config_t timer_cfg;
    ledc_channel_config_t ch_cfg;
    esp_err_t rc;

    if (s_dctx.initialized) {
        ESP_LOGW(s_TAG, "Driver ja inicializado.");
        return ESP_ERR_INVALID_STATE;
    }

    /* Configurar o timer do LEDC */
    timer_cfg.speed_mode       = LEDC_SPEED_MODE;
    timer_cfg.timer_num        = LEDC_TIMER;
    timer_cfg.duty_resolution  = LEDC_DUTY_RES;
    timer_cfg.freq_hz          = 523;         /* Frequencia inicial (nota C4) */
    timer_cfg.clk_cfg          = LEDC_USE_REF_TICK;

#ifdef CONFIG_IDF_TARGET_ESP32S3
    timer_cfg.clk_cfg = LEDC_AUTO_CLK;
#endif
    rc = ledc_timer_config(&timer_cfg);
    if (rc != ESP_OK) {
        ESP_LOGE(s_TAG, "Erro ao configurar o timer LEDC. rc: %d", rc);
        return rc;
    }

    /* Configurar o canal do LEDC */
    ch_cfg.speed_mode       = LEDC_SPEED_MODE;
    ch_cfg.channel          = LEDC_CHANNEL;
    ch_cfg.timer_sel        = LEDC_TIMER;
    ch_cfg.intr_type  = LEDC_INTR_DISABLE;
    ch_cfg.gpio_num   = BUZZER_GPIO;
    ch_cfg.duty       = 0;


    rc = ledc_channel_config(&ch_cfg);
    if (rc != ESP_OK) {
        ESP_LOGE(s_TAG, "Erro ao configurar o canal LEDC. rc: %d", rc);
        return rc;
    }

    s_dctx.freq_hz     = 523;
    s_dctx.current_dut = 0;
    s_dctx.initialized = true;

    ESP_LOGI(s_TAG, "Driver inicializado (GPIO %d, freq %.0f Hz).",
             BUZZER_GPIO, (float)s_dctx.freq_hz);
    return ESP_OK;
}

esp_err_t LEDBuzzerSet(char value, int freq) {
    esp_err_t rc;

    if (!s_dctx.initialized) {
        ESP_LOGE(s_TAG, "Driver nao inicializado. Chame LEDBuzzerInit() primeiro.");
        return ESP_ERR_INVALID_STATE;
    }

    /* Validar intervalo de frequencia (buzzer passivo: 523 Hz – 20 kHz).
     * so valida quando value > 0. */
    if (value > 0 && (freq < 523 || freq > 20000)) {
        ESP_LOGE(s_TAG, "Frequencia invalida: %d Hz. Range: 523-20000.", freq);
        return ESP_ERR_INVALID_ARG;
    }

    if (value > 0) {
        ESP_LOGI(s_TAG, "Ligar buzzer a %.0f Hz.", (float)freq);
        /* Set duty ~50% se ainda nao foi setado */
        uint32_t duty = s_dctx.current_dut ? s_dctx.current_dut : (1 << 7);
        s_LedcBuzzerSetDuty(duty);

        rc = ledc_set_freq(LEDC_SPEED_MODE, LEDC_TIMER, freq);
        if (rc != ESP_OK) {
            ESP_LOGE(s_TAG, "Erro ao configurar frequencia. rc: %d", rc);
            return rc;
        }
        s_dctx.freq_hz     = freq;
        s_dctx.current_dut = duty;
    } else {
        ESP_LOGI(s_TAG, "Desligar buzzer.");
        s_LedcBuzzerSetDuty(0);
        s_dctx.current_dut = 0;
    }

    return ESP_OK;
}

esp_err_t LEDBuzzerPulse(unsigned period, unsigned duty_cycle) {
    esp_err_t rc;

    if (!s_dctx.initialized) {
        ESP_LOGE(s_TAG, "Driver nao inicializado. Chame LEDBuzzerInit() primeiro.");
        return ESP_ERR_INVALID_STATE;
    }

    /* Clamp duty_cycle */
    if (duty_cycle > 100) {
        duty_cycle = 100;
    }

    uint32_t raw_dut = (uint32_t)duty_cycle * ((1U << LEDC_DUTY_RES) - 1) / 100;
    s_LedcBuzzerSetDuty(raw_dut);
    s_dctx.current_dut = raw_dut;

    if (period == 0) {
        /* Desligar buzzer */
        ESP_LOGI(s_TAG, "Pulse com period = 0 → desliga buzzer.");
        s_LedcBuzzerSetDuty(0);
        s_dctx.current_dut = 0;
        return ESP_OK;
    }

    ESP_LOGI(s_TAG, "Pulse: period %u ms, duty %u %% (%lu bits).",
             period, duty_cycle, (unsigned long)raw_dut);
    vTaskDelay(pdMS_TO_TICKS(period));

    /* Ao fim do pulso: parar */
    s_LedcBuzzerSetDuty(0);
    s_dctx.current_dut = 0;

    return ESP_OK;
}

static void s_LedcBuzzerSetDuty(uint32_t duty) {
    ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL);
}


