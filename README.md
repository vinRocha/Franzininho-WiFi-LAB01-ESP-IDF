# Franzininho WiFi LAB01 - ESP-IDF

Este repositório contem drivers e exemplos de uso para os periféricos encontrados na placa de desenvolvimento [Franzininho WiFi LAB01](https://docs.franzininho.com.br/docs/franzininho-wifi-lab01/) do projeto [Franzininho](https://franzininho.com.br/), desenvolvidos com [esp-idf v5.4.1](https://docs.espressif.com/projects/esp-idf/en/v5.4.1/esp32s2/index.html)

# Executando os exemplos

 1. Tenha em mãos uma placa de desenvolvimento Franzininho WiFi com cabo USB para flash do dispositivo.

 2. Caso não tenha o ambiente de desenvolvimento (esp-idf) configurado, siga o tutorial em [Primeiros Passos com ESP-IDF](https://docs.franzininho.com.br/docs/franzininho-wifi/exemplos-espidf/primeiros-passos).

 3. Clone este repositório:

         $ git clone https://github.com/Franzininho/Franzininho-WiFi-LAB01-ESP-IDF.git

4. Acesse o diretório de uma aplicação de exemplo:

        $ cd Franzininho-WiFi-LAB01-ESP-IDF/exemplos/dht11_exemplo/

5. Com o ambiente do esp-idf corretamente configurado (ver passo 2), compile e execute a aplicação

        $ idf.py -p /dev/ttyACM0 build flash monitor

    **OBS.:** Lembre-se de alterar o parametro "/dev/ttyACM0" para a porta serial na qual o Franzininho WiFi aparece conectado em seu sistema. Em caso de duvidas, siga o tutorial fornecido no passo 2 correspondente ao seu sistema operacional de preferencia.

