#include <stdio.h>
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SERVO_PIN 18           // GPIO pin for servo control
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT  // 13-bit resolution
#define LEDC_FREQUENCY 50                 // 50 Hz for servo

// Initialize servo
void servo_init() {
    // Configure LEDC timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // Configure LEDC channel
    ledc_channel_config_t ledc_channel = {
        .gpio_num = SERVO_PIN,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel);
}

// Jump to specific angle (0-180 degrees)
void jump_to_angle(int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    
    // Map angle to PWM duty cycle
    // Servo PWM: 1ms (1000µs) = 0°, 2ms (2000µs) = 180°
    // At 50Hz, period = 20ms = 20000µs
    // Duty cycle = (pulse_width_µs / 20000) * (2^13 - 1)
    int pulse_width_us = 1000 + (angle * 1000 / 180);  // 1000-2000 µs
    int duty = (pulse_width_us * 8192) / 20000;  // 8192 = 2^13
    
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

// Cleanup
void servo_cleanup() {
    ledc_stop(LEDC_MODE, LEDC_CHANNEL, 0);
}

// Test function
int servoTest() {
    printf("Servo Control Test\n");
    servo_init();
    
    // Test: sweep from 0 to 180 degrees
    printf("Moving to 0 degrees\n");
    jump_to_angle(0);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    printf("Moving to 90 degrees\n");
    jump_to_angle(90);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    printf("Moving to 180 degrees\n");
    jump_to_angle(180);
    vTaskDelay(pdMS_TO_TICKS(1000));

    printf("Moving back to 90 degrees\n");
    jump_to_angle(90);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    servo_cleanup();
    return 0;
}