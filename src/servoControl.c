#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "iot_servo.h"

#define SERVO_CH0_PIN 22           // GPIO pin for servo control

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0

#define ANGLE_LIMIT_MIN 45.0
#define ANGLE_LIMIT_MAX 135.0
#define ANGLE_INIT 90.0

typedef float servoAngle;

static const char *TAG = "ServoControl";

// Function Declarations
void servoInit();
void jumpToAngle(servoAngle angle);
void getCurrentAngle(servoAngle *angle);
void servoCleanup();

// Initialize servo
void servoInit() {
    servo_config_t servo_cfg = {
        .max_angle = 180,
        .min_width_us = 500,    // 1500 - 10 * 90 = 600 (+ margin)
        .max_width_us = 2500,   // 1500 + 10 * 90 = 2400 (+ margin)
        .freq = 50,
        .timer_number = LEDC_TIMER_0,
        .channels = {
            .servo_pin = {
                SERVO_CH0_PIN,
            },
            .ch = {
                LEDC_CHANNEL_0,
            },
        },
        .channel_number = 1,
    };
    esp_err_t err = iot_servo_init(LEDC_LOW_SPEED_MODE, &servo_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "iot_servo_init failed: %d", err);
    }
    iot_servo_write_angle(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0.0f); // Start at 0 degrees
}

void jumpToAngle(servoAngle angle) {
    if (angle < ANGLE_LIMIT_MIN) {
        ESP_LOGW(TAG, "Clip (%.2f) to min angle (%.2f)", angle, ANGLE_LIMIT_MIN);
        angle = ANGLE_LIMIT_MIN;
    }
    
    if (angle > ANGLE_LIMIT_MAX) {
        ESP_LOGW(TAG, "Clip (%.2f) to max angle (%.2f)", angle, ANGLE_LIMIT_MAX);
        angle = ANGLE_LIMIT_MAX;
    }

    iot_servo_write_angle(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, angle);
}

void getCurrentAngle(servoAngle *angle) {
    iot_servo_read_angle(LEDC_MODE, LEDC_CHANNEL, angle);
}

// Cleanup
void servoCleanup() {
    iot_servo_deinit(LEDC_LOW_SPEED_MODE);
}

// Test function
int servoTest() {
    printf("Servo Control Test\n");
    servoInit();
    
    servoAngle angle = 0.0;
    servoAngle angle_step = 90.0; // Step 90 degrees at a time
    servoAngle dir = 1.0;
    while (1) {
        if (angle >= 180.0) {
            dir = -1.0;
        }
        else if (angle <= 0.0) {
            dir = 1.0;
        }
        angle += dir * angle_step;
        jumpToAngle(angle);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    servoCleanup();
    return 0;
}