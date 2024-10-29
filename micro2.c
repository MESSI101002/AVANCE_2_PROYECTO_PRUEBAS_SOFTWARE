#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/semphr.h"


#define LED1_PIN GPIO_NUM_21
#define LED2_PIN GPIO_NUM_22
#define LED3_PIN GPIO_NUM_23
#define SERVO_PIN GPIO_NUM_18

SemaphoreHandle_t servoSemaphore;

void configure_leds(void) {
    esp_rom_gpio_pad_select_gpio(LED1_PIN);
    esp_rom_gpio_pad_select_gpio(LED2_PIN);
    esp_rom_gpio_pad_select_gpio(LED3_PIN);
    gpio_set_direction(LED1_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED2_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED3_PIN, GPIO_MODE_OUTPUT);
}

void configure_pwm_servo(void) {
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 50,  // Frecuencia de PWM para servomotores (50 Hz)
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t channel = {
        .gpio_num = SERVO_PIN,  // Pin para controlar el servomotor
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,  // Inicialmente apagado
        .hpoint = 0
    };
    ledc_channel_config(&channel);
}

void control_leds(void *pvParameter) {
    configure_leds();  // Configurar pines de los LEDs

    while (1) {
        // Encender LED1
        gpio_set_level(LED1_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(10000));  // Esperar 5 segundos

        // Encender LED2
        gpio_set_level(LED1_PIN, 0);  // Apagar LED1
        gpio_set_level(LED2_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(2000));  // Esperar 0.7 segundos

        // Encender LED3
        gpio_set_level(LED2_PIN, 0);  // Apagar LED2
        gpio_set_level(LED3_PIN, 1);

        // Notificar a la tarea del servo que puede comenzar
        xSemaphoreGive(servoSemaphore);

        vTaskDelay(pdMS_TO_TICKS(4000));  // Esperar 2 segundos

        // Apagar todos los LEDs
        gpio_set_level(LED1_PIN, 0);
        gpio_set_level(LED2_PIN, 0);
        gpio_set_level(LED3_PIN, 0);

        vTaskDelay(pdMS_TO_TICKS(10000));  // Esperar 1 segundo antes de empezar de nuevo
    }
}

void control_servo(void *pvParameter) {
    configure_pwm_servo();  // Configurar PWM para el servomotor

    while (1) {
        // Esperar la señal para comenzar
        xSemaphoreTake(servoSemaphore, portMAX_DELAY);

        // Probar diferentes valores de duty cycle
        for (int duty = 500; duty <= 2500; duty += 500) {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            vTaskDelay(pdMS_TO_TICKS(3000));  // Esperar 1 segundo
        }

        // Volver al inicio
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        vTaskDelay(pdMS_TO_TICKS(100));  // Esperar 1 segundo
    }
}

void app_main(void) {
    servoSemaphore = xSemaphoreCreateBinary();

    xTaskCreate(&control_leds, "led_task", 2048, NULL, 5, NULL);
    xTaskCreate(&control_servo, "servo_task", 2048, NULL, 5, NULL);
}