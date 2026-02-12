#include <stdio.h>
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SERVO_PIN 23           // GPIO pin for servo control
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT  // 13-bit resolution
#define LEDC_FREQUENCY 50                 // 50 Hz for servo
#define STEP_MAX_DUTY_CYCLE_INC 10
#define MIN_DUTY_CYCLE angle_to_duty_cycle(0)
#define MAX_DUTY_CYCLE angle_to_duty_cycle(180)

// Function Declarations
void servo_init();
void glide_to_angle(int end_angle, int duration_ms);
void jump_to_angle(int angle);
int angle_to_duty_cycle(float angle);
float duty_cycle_to_angle(int duty);
void step_once(bool dirnClockwise);
void servo_cleanup();
void set_duty_cycle(int duty);
void set_current_duty(int duty);
int get_current_duty();

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
    set_duty_cycle(angle_to_duty_cycle(90));  // Start at 90 degrees
}

void glide_to_angle(int end_angle, int duration_ms) {
    // get current duty cycle
    int target_duty_cycle = angle_to_duty_cycle(end_angle);
    int duty_diff = target_duty_cycle - get_current_duty();
    int move_steps = abs(duty_diff) / STEP_MAX_DUTY_CYCLE_INC;
    bool dir_clockwise = (duty_diff > 0);

    for (int step = 1; step <= move_steps; step++) {
        if (step == move_steps) {
            // Final step, set to exact target duty cycle
            set_duty_cycle(target_duty_cycle);
        }
        else {
            step_once(dir_clockwise);
            vTaskDelay(pdMS_TO_TICKS(duration_ms / move_steps));
        }
    }
}

int angle_to_duty_cycle(float angle) {
    // Map angle to PWM duty cycle
    // Servo PWM: 1ms (1000µs) = 0°, 2ms (2000µs) = 180°
    // At 50Hz, period = 20ms = 20000µs
    // Duty cycle = (pulse_width_µs / 20000) * (2^13 - 1)
    int pulse_width_us = 1000 + (angle * 1000 / 180);  // 1000-2000 µs
    int duty = (pulse_width_us * 8192) / 20000;  // 8192 = 2^13
    return duty;
}

float duty_cycle_to_angle(int duty) {
    int pulse_width_us = (duty * 20000) / 8192;
    float angle = ((pulse_width_us - 1000) * 180) / 1000;
    return angle;
}

void step_once(bool dirnClockwise) {    
    int step_increment = STEP_MAX_DUTY_CYCLE_INC;
    if (!dirnClockwise) {
        step_increment = -STEP_MAX_DUTY_CYCLE_INC;
    }
    
    int new_duty = get_current_duty() + step_increment;
    if (new_duty < 0) new_duty = 0;
    if (new_duty > MAX_DUTY_CYCLE) new_duty = MAX_DUTY_CYCLE;
    
    set_duty_cycle(new_duty);
}

// Jump to specific angle (0-180 degrees)
void jump_to_angle(int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    int duty = angle_to_duty_cycle(angle);
    
    set_duty_cycle(duty);
}

void set_duty_cycle(int duty) {
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

int get_current_duty() {
    return ledc_get_duty(LEDC_MODE, LEDC_CHANNEL);
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
    printf("Glide to 0 degrees\n");
    glide_to_angle(0, 1000);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    printf("Glide to 90 degrees\n");
    glide_to_angle(90, 500);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    printf("Glide to 145 degrees\n");
    glide_to_angle(145, 750);
    vTaskDelay(pdMS_TO_TICKS(1000));

    printf("Glide back to 90 degrees\n");
    glide_to_angle(90, 200);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    servo_cleanup();
    return 0;
}