/*
 * SPDX-License-Identifier: MIT
 *
 */

/**
 * @file buzzer_exemplo.c
 *
 * @brief Aplicacao exemplo para controlar o buzzer.
 *
 * @author Vinicius Silva <silva.viniciusr@gmail.com>
 *
 * @date 29 de Maio de 2025
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "buzzer_driver.h"

/**
 * @brief Loop principal
 *
 * Nao deve retornar.
 *
 */
void app_main(void) {

  fprintf(stdout, "Executando buzzer_exemplo.c\n"
                  "Confirme que o jumper de ativacao do buzzer "
                  "encontra-se instalado");

  /* Inicia o driver do buzzer */
  if (BuzzerInit() != ESP_OK) {
    fprintf(stderr, "Erro ao inicializar o driver do buzzer...\n");
    for(;;){}
  }

  /* Loop infinito da aplicacao */
  for (;;) {
    /* Ativa o buzzer por 0.5s e desativa por 1.5s
     * Repete por 4 vezes */
    for (int i = 0; i < 4; i++){
      BuzzerSet(1);
      vTaskDelay(500 / portTICK_PERIOD_MS);

      BuzzerSet(0);
      vTaskDelay(1500 / portTICK_PERIOD_MS);
    }

    /* Configura buzzer no modo beat com periodo de 0.6s
     * e duty_cycle de 40%. 
     *
     * Nesse modo nao precisamos nos preocupar em ligar e 
     * desligar o buzzer via BuzzerSet() */
    BuzzerBeat(300, 50);

    /* Aguarda 8 repeticoes do modo beat */
    vTaskDelay(300 * 8 / portTICK_PERIOD_MS);

    /* Desativa o modo beat */
    BuzzerSet(0);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }

  /* Nao deve chegar aqui!! */
  return;
}
