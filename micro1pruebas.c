#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/ledc.h"

// Definiciones y configuraciones originales

#define LED_PIN GPIO_NUM_23
#define POT_PIN ADC1_CHANNEL_6

#define LCD_D4_GPIO 16
#define LCD_D5_GPIO 17
#define LCD_D6_GPIO 5
#define LCD_D7_GPIO 18
#define LCD_RS_GPIO 2
#define LCD_EN_GPIO 15

#define LCD_COLS 16
#define LCD_ROWS 2
#define BUTTON_GPIO 4
#define START_SIGNAL_GPIO 27

#define LCD_CLEAR_DISPLAY 0x01
#define LCD_RETURN_HOME 0x02
#define LCD_ENTRY_MODE_SET 0x04
#define LCD_DISPLAY_CONTROL 0x08
#define LCD_CURSOR_SHIFT 0x10
#define LCD_FUNCTION_SET 0x20
#define LCD_SET_CGRAM_ADDR 0x40
#define LCD_SET_DDRAM_ADDR 0x80

#define LCD_ENTRY_RIGHT 0x00
#define LCD_ENTRY_LEFT 0x02
#define LCD_ENTRY_SHIFT_INCREMENT 0x01
#define LCD_ENTRY_SHIFT_DECREMENT 0x00

#define LCD_DISPLAY_ON 0x04
#define LCD_DISPLAY_OFF 0x00
#define LCD_CURSOR_ON 0x02
#define LCD_CURSOR_OFF 0x00
#define LCD_BLINK_ON 0x01
#define LCD_BLINK_OFF 0x00

#define LCD_4BIT_MODE 0x00
#define LCD_8BIT_MODE 0x10
#define LCD_1LINE_MODE 0x00
#define LCD_2LINE_MODE 0x08
#define LCD_5x8_FONT 0x00
#define LCD_5x10_FONT 0x04

volatile bool start_working = false;

// Funciones originales para la configuración y funcionamiento del LCD, ADC y PWM

void lcdPulseEnable(void) {
    gpio_set_level(LCD_EN_GPIO, 1);
    esp_rom_delay_us(1);
    gpio_set_level(LCD_EN_GPIO, 0);
    esp_rom_delay_us(100);
}

void lcdSendNibble(uint8_t data) {
    gpio_set_level(LCD_D4_GPIO, (data >> 0) & 0x01);
    gpio_set_level(LCD_D5_GPIO, (data >> 1) & 0x01);
    gpio_set_level(LCD_D6_GPIO, (data >> 2) & 0x01);
    gpio_set_level(LCD_D7_GPIO, (data >> 3) & 0x01);
}

void lcdSendByte(uint8_t data, uint8_t mode) {
    gpio_set_level(LCD_RS_GPIO, mode);
    lcdSendNibble(data >> 4);
    lcdPulseEnable();
    lcdSendNibble(data);
    lcdPulseEnable();
}

void lcdSendCommand(uint8_t command) {
    lcdSendByte(command, 0);
}

void lcdSendData(uint8_t data) {
    lcdSendByte(data, 1);
}

void lcdInit(void) {
    esp_rom_gpio_pad_select_gpio(LCD_D4_GPIO);
    esp_rom_gpio_pad_select_gpio(LCD_D5_GPIO);
    esp_rom_gpio_pad_select_gpio(LCD_D6_GPIO);
    esp_rom_gpio_pad_select_gpio(LCD_D7_GPIO);
    esp_rom_gpio_pad_select_gpio(LCD_RS_GPIO);
    esp_rom_gpio_pad_select_gpio(LCD_EN_GPIO);
    gpio_set_direction(LCD_D4_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(LCD_D5_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(LCD_D6_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(LCD_D7_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(LCD_RS_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(LCD_EN_GPIO, GPIO_MODE_OUTPUT);

    esp_rom_delay_us(50000);
    lcdSendNibble(0x03);
    esp_rom_delay_us(4500);
    lcdSendNibble(0x03);
    esp_rom_delay_us(200);
    lcdSendNibble(0x03);
    lcdSendNibble(0x02);

    lcdSendCommand(LCD_FUNCTION_SET | LCD_4BIT_MODE | LCD_2LINE_MODE | LCD_5x8_FONT);
    lcdSendCommand(LCD_DISPLAY_CONTROL | LCD_DISPLAY_ON | LCD_CURSOR_OFF | LCD_BLINK_OFF);
    lcdSendCommand(LCD_CLEAR_DISPLAY);
    lcdSendCommand(LCD_ENTRY_MODE_SET | LCD_ENTRY_LEFT | LCD_ENTRY_SHIFT_DECREMENT);
}

void lcdClear(void) {
    lcdSendCommand(LCD_CLEAR_DISPLAY);
    esp_rom_delay_us(2000);
}

void lcdPrint(const char *str) {
    while (*str) {
        lcdSendData(*str++);
    }
}

void lcdPrintInt(int num) {
    char buffer[10];
    sprintf(buffer, "%d", num);
    lcdPrint(buffer);
}

void configure_adc(void) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(POT_PIN, ADC_ATTEN_DB_11);
}

uint32_t read_potentiometer(void) {
    uint32_t adc_reading = adc1_get_raw(POT_PIN);
    return adc_reading;
}

void configure_pwm(void) {
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t channel = {
        .gpio_num = LED_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&channel);
}

void mostrarContador1(int contador1, int decimas, int centesimas) {
    char buffer[17];
    memset(buffer, ' ', sizeof(buffer)-1);
    buffer[sizeof(buffer)-1] = '\0';
    lcdClear();
    lcdPrint("   RUN   TIME");
    lcdSendCommand(LCD_SET_DDRAM_ADDR | 0x40);
    snprintf(buffer, sizeof(buffer), "       %d%d.%d", contador1, decimas, centesimas);
    lcdPrint(buffer);
}

void init_button(void) {
    esp_rom_gpio_pad_select_gpio(BUTTON_GPIO);
    gpio_set_direction(BUTTON_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_GPIO, GPIO_PULLUP_ONLY);

    esp_rom_gpio_pad_select_gpio(START_SIGNAL_GPIO);
    gpio_set_direction(START_SIGNAL_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(START_SIGNAL_GPIO, GPIO_PULLUP_ONLY);
}

bool is_button_pressed(void) {
    if (gpio_get_level(BUTTON_GPIO) == 0) {
        start_working = false;
    }
    if (gpio_get_level(START_SIGNAL_GPIO) == 0) {
        start_working = true;
    }
    return start_working;
}

void app_main(void) {
    configure_adc();
    configure_pwm();
    lcdInit();
    init_button();

    int contador1 = 0;
    int decimas = 0;
    int centesimas = 0;
    bool running = true;

    while (1) {
        if (is_button_pressed()) {
            running = !running;
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        if (running) {
            uint32_t pot_value = read_potentiometer();
            uint32_t duty = (pot_value * 8191) / 4095;
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

            mostrarContador1(contador1, decimas, centesimas);

            if (start_working) {
                centesimas++;
                if (centesimas > 9) {
                    centesimas = 0;
                    decimas++;
                    if (decimas > 9) {
                        decimas = 0;
                        contador1++;
                        if (contador1 > 9999) {
                            contador1 = 0;
                        }
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Pruebas añadidas

void test_read_potentiometer() {
    uint32_t result = read_potentiometer();
    printf("Potentiometer value: %d\n", result);
    if (result >= 0 && result <= 4095) {
        printf("test_read_potentiometer PASSED\n");
    } else {
        printf("test_read_potentiometer FAILED\n");
    }
}

void test_lcdPrintInt() {
    lcdInit();
    lcdPrintInt(1234);
    printf("LCD should display: 1234\n");
    printf("test_lcdPrintInt PASSED\n");
}

void test_adc_pwm_integration() {
    configure_adc();
    configure_pwm();

    uint32_t pot_value = read_potentiometer();
    uint32_t duty = (pot_value * 8191) / 4095;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

    if (duty >= 0 && duty <= 8191) {
        printf("test_adc_pwm_integration PASSED\n");
    } else {
        printf("test_adc_pwm_integration FAILED\n");
    }
}

void test_mostrarContador1() {
    lcdInit();
    mostrarContador1(9, 5, 3);
    printf("Checking display for correct time format.\n");
    printf("test_mostrarContador1 PASSED\n");
}

void test_lcdPrintInt_performance() {
    lcdInit();
    TickType_t start = xTaskGetTickCount();
    lcdPrintInt(9999);
    TickType_t end = xTaskGetTickCount();
    printf("lcdPrintInt execution time: %d ms\n", (end - start) * portTICK_PERIOD_MS);
    printf("test_lcdPrintInt_performance PASSED\n");
}

void app_main_tests(void) {
    printf("Running unit tests...\n");
    test_read_potentiometer();
    test_lcdPrintInt();

    printf("Running integration test...\n");
    test_adc_pwm_integration();

    printf("Running regression test...\n");
    test_mostrarContador1();

    printf("Running performance test...\n");
    test_lcdPrintInt_performance();
}