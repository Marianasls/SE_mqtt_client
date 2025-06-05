/**
 * @author: Mariana Santos
 * Sistema de Alerta de enchentes
 */
#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"

#include "lib/font.h"
#include "lib/ssd1306.h"
#include "lib/led_matrix.h"
#include "lib/ws2812.pio.h"

#define JOYSTICK_X_ADC 0
#define JOYSTICK_Y_ADC 1
#define BUZZER_A 21
#define LED_R 13
#define LED_B 12
#define LED_G 11
#define WS2812_PIN 7
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define address 0x3C
#define ADC_PIN_Y 27
#define ADC_PIN_X 26
#define OLED_TASK_PRIORITY 2
#define ALERT_TASK_PRIORITY 2
#define SENSOR_TASK_PRIORITY 1
#define PWM_DIVISER 20
#define PWM_WRAP 2000 

// Trecho para modo BOOTSEL com botão B
#include "pico/bootrom.h"
#define buttonB 6

void gpio_irq_handler(uint gpio, uint32_t events)
{
    reset_usb_boot(0, 0);
}

// Variáveis globais
ssd1306_t ssd;
QueueHandle_t dataQueue;
int mode = 0;
typedef struct {
    float nivel_agua;
    float volume_chuva;
} SensorData;

// Declaração de funções
void setup_gpio();
float read_sensor(uint8_t channel);
void readSensorsTask(void *pvParameters);
void displayTask(void *pvParameters);
void alertTask(void *pvParameters);
void _pwm_init(int pin);

int main() {
    stdio_init_all();

    // Inicializar a matriz de LEDs (WS2812)
    PIO pio = pio0;
    int sm = 0;
    uint offset = pio_add_program(pio, &ws2812_program);
    ws2812_program_init(pio, sm, offset, WS2812_PIN, 800000, false);

    // Inicializa pinos GPIO
    setup_gpio();

    // Inicializa o display OLED
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, address, I2C_PORT); // Inicializa o display
    ssd1306_config(&ssd);                                         // Configura o display
    ssd1306_send_data(&ssd);                                      // Envia os dados para o display

    // Limpa o display. O display inicia com todos os pixels apagados.
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

    ssd1306_draw_string(&ssd, "Monitoramento", 2, 2);
    ssd1306_draw_string(&ssd, "de enchente", 2, 12);


    dataQueue = xQueueCreate(5, sizeof(SensorData));

    xTaskCreate(readSensorsTask, "Sensor", 256, NULL, SENSOR_TASK_PRIORITY, NULL);
    xTaskCreate(displayTask, "Display", 512, NULL, OLED_TASK_PRIORITY, NULL);
    xTaskCreate(alertTask, "Alerta", 256, NULL, ALERT_TASK_PRIORITY, NULL);

    vTaskStartScheduler();

}

void _pwm_init(int pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(pin);
    pwm_set_clkdiv(slice_num, PWM_DIVISER);
    pwm_set_wrap(slice_num, PWM_WRAP);

    pwm_set_gpio_level(pin, 0);
    pwm_set_enabled(slice_num, true);
}

void setup_gpio() {
    // Para ser utilizado o modo BOOTSEL com botão B
    gpio_init(buttonB);
    gpio_set_dir(buttonB, GPIO_IN);
    gpio_pull_up(buttonB);
    gpio_set_irq_enabled_with_callback(buttonB, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    gpio_init(LED_R); gpio_set_dir(LED_R, GPIO_OUT);
    gpio_init(LED_G); gpio_set_dir(LED_G, GPIO_OUT);
    gpio_init(LED_B); gpio_set_dir(LED_B, GPIO_OUT);

    _pwm_init(BUZZER_A); // Inicializa o PWM no pino do buzzer

    // Inicializa os pinos do display
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Inicializa o ADC
    adc_gpio_init(ADC_PIN_X); adc_gpio_init(ADC_PIN_Y);  
    adc_init();
}

float read_sensor(uint8_t channel) {
    adc_select_input(channel);
    float media = 0.0f;
    for (int i = 0; i < 500; i++){
        media += adc_read();
    }
    media /= 500.0f;
    return media * (100.0f / 4095.0f);
}

void readSensorsTask(void *pvParameters) {
    SensorData data;
    while (1) {
        data.volume_chuva = read_sensor(JOYSTICK_X_ADC);
        data.nivel_agua = read_sensor(JOYSTICK_Y_ADC);

        xQueueSend(dataQueue, &data, 0);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void displayTask(void *pvParameters) {
    SensorData data;
    while (1) {
        if (xQueueReceive(dataQueue, &data, portMAX_DELAY)) {
            // Atualiza o display OLED
            char buffer[20];
            snprintf(buffer, 20, "\nAgua: %.2f%%\n", data.nivel_agua);
            printf(buffer);
            ssd1306_draw_string(&ssd, buffer, 2, 30);

            snprintf(buffer, 20, "Chuva: %.2f%%\n", data.volume_chuva);
            printf(buffer);
            ssd1306_draw_string(&ssd, buffer, 2, 40);

            ssd1306_send_data(&ssd);
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void alertTask(void *pvParameters) {
    SensorData data;
    while (1) {
        if (xQueueReceive(dataQueue, &data, portMAX_DELAY)) {
            bool alerta = (data.nivel_agua >= 70 || data.volume_chuva >= 80);

            if (alerta) {
                mode = 1;
                // LED RGB: vermelho
                gpio_put(LED_R, 1);
                gpio_put(LED_G, 0);
                gpio_put(LED_B, 0);

                // Buzzer: som de alerta
                pwm_set_gpio_level(BUZZER_A, PWM_WRAP / 8);
      
                // Matriz de LEDs exibe símbolo de alerta 
                draw_triangle();
                set_leds(255, 0, 0);  // Cor vermelha

                ssd1306_draw_string(&ssd, "ALERTA", 2, 55);
            } else {
                // LED RGB: verde
                gpio_put(LED_R, 0);
                gpio_put(LED_G, 1);
                gpio_put(LED_B, 0);

                pwm_set_gpio_level(BUZZER_A, 0); 
                clear_buffer();
                set_leds(0, 0, 0);
                if(mode){
                    ssd1306_fill(&ssd, false);
                    ssd1306_send_data(&ssd);
                    ssd1306_draw_string(&ssd, "Monitoramento", 2, 2);
                    ssd1306_draw_string(&ssd, "de enchente", 2, 12);
                    mode = 0;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
}

