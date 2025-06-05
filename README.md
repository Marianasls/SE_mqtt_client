# Cliente MQTT para projeto IoT com a placa BitDogLab

Cliente MQTT para Raspberry Pi Pico W  
**Publica valores de sensores em `/agua` e `/chuva` e recebe alertas em `/alerta`**

---

## Descrição

Este projeto implementa um cliente MQTT embarcado para a Raspberry Pi Pico W, capaz de:

- **Publicar** valores lidos do joystick:
  - Eixo X → tópico `/chuva`
  - Eixo Y → tópico `/agua`
- **Assinar** o tópico `/alerta`:
  - Ao receber `"1"`, acende o LED RGB vermelho e ativa o buzzer.
  - Qualquer outro valor desliga o LED e o buzzer.

O código utiliza a stack LWIP, suporte a TLS (opcional) e é baseado nos exemplos oficiais da Raspberry Pi.

---

## Hardware necessário

- Raspberry Pi Pico W
- Joystick analógico (conectado aos ADCs)
- LED RGB (pino R: 13)
- Buzzer (pino 21)
- Conexão Wi-Fi

---

## Configuração

Edite as seguintes definições no início do arquivo `SE_mqtt_client.c`:

```c
#define WIFI_SSID "sua_rede_wifi"
#define WIFI_PASSWORD "sua_senha_wifi"
#define MQTT_SERVER "ip_do_broker"
#define MQTT_USERNAME "usuario"      
#define MQTT_PASSWORD "senha"        