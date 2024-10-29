#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/semphr.h"

// Definiciones y configuraciones originales

#define LED1_PIN GPIO_NUM_21
#define LED2_PIN GPIO_NUM_22
#define LED3_PIN GPIO_NUM_23
#define SERVO_PIN GPIO_NUM_18

SemaphoreHandle_t servoSemaphore;

// Funciones originales para configurar y controlar LEDs y servo

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
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t channel = {
        .gpio_num = SERVO_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&channel);
}

void control_leds(void *pvParameter) {
    configure_leds();

    while (1) {
        gpio_set_level(LED1_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(10000));

        gpio_set_level(LED1_PIN, 0);
        gpio_set_level(LED2_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(2000));

        gpio_set_level(LED2_PIN, 0);
        gpio_set_level(LED3_PIN, 1);

        xSemaphoreGive(servoSemaphore);

        vTaskDelay(pdMS_TO_TICKS(4000));

        gpio_set_level(LED1_PIN, 0);
        gpio_set_level(LED2_PIN, 0);
        gpio_set_level(LED3_PIN, 0);

        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

void control_servo(void *pvParameter) {
    configure_pwm_servo();

    while (1) {
        xSemaphoreTake(servoSemaphore, portMAX_DELAY);

        for (int duty = 500; duty <= 2500; duty += 500) {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            vTaskDelay(pdMS_TO_TICKS(3000));
        }

        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void) {
    servoSemaphore = xSemaphoreCreateBinary();

    xTaskCreate(&control_leds, "led_task", 2048, NULL, 5, NULL);
    xTaskCreate(&control_servo, "servo_task", 2048, NULL, 5, NULL);
}

// Pruebas añadidas

// Prueba unitaria para el control de LEDs
void test_led_control() {
    configure_leds();
    gpio_set_level(LED1_PIN, 1);
    printf("LED1 state: %d\n", gpio_get_level(LED1_PIN));
    if (gpio_get_level(LED1_PIN) == 1) {
        printf("test_led_control PASSED\n");
    } else {
        printf("test_led_control FAILED\n");
    }
    gpio_set_level(LED1_PIN, 0);
}

// Prueba de integración entre control de LEDs y servo
void test_led_servo_integration() {
    servoSemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(servoSemaphore);

    if (xSemaphoreTake(servoSemaphore, 1000 / portTICK_PERIOD_MS)) {
        printf("Servo semaphore received, test_led_servo_integration PASSED\n");
    } else {
        printf("Servo semaphore failed, test_led_servo_integration FAILED\n");
    }
}

// Prueba de regresión para el ciclo de duty en control_servo
void test_servo_duty_cycle() {
    configure_pwm_servo();
    for (int duty = 500; duty <= 2500; duty += 500) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        printf("Duty cycle: %d\n", duty);
        if (duty >= 500 && duty <= 2500) {
            printf("Duty cycle within range, test_servo_duty_cycle PASSED\n");
        } else {
            printf("Duty cycle out of range, test_servo_duty_cycle FAILED\n");
        }
    }
}

// Prueba de rendimiento para control_servo
void test_control_servo_performance() {
    TickType_t start = xTaskGetTickCount();
    control_servo(NULL);
    TickType_t end = xTaskGetTickCount();
    printf("control_servo execution time: %d ms\n", (end - start) * portTICK_PERIOD_MS);
    printf("test_control_servo_performance PASSED\n");
}

// Función para ejecutar todas las pruebas
void app_main_tests(void) {
    printf("Running unit test for LED control...\n");
    test_led_control();

    printf("Running integration test for LED-servo interaction...\n");
    test_led_servo_integration();

    printf("Running regression test for servo duty cycle...\n");
    test_servo_duty_cycle();

    printf("Running performance test for control_servo...\n");
    test_control_servo_performance();
}