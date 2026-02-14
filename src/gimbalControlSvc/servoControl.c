#include "servoControl.h"

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "iot_servo.h"

#define SERVO_CH0_PIN 22           // GPIO pin for servo control

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0

static const char *servoTAG = "  -- servoControl";
static float currentTiltAngle = TILT_ANGLE_INIT;

// Function Declarations
void tiltToAngle(float angle);
void tiltDegrees(float degrees);
float getCurrentAngle(void);
void servoCleanup();
void setServoAngle(float angle);

// Initialize servo
void servoControl_init() {
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
        ESP_LOGE(servoTAG, "iot_servo_init failed: %d", err);
    }
    setServoAngle(TILT_ANGLE_INIT); // Start at 0 degrees
}

void tiltToAngle(float angle) {
    if (angle < TILT_ANGLE_LIMIT_MIN) {
        ESP_LOGW(servoTAG, "Clip (%.2f) to min angle (%.2f)", angle, TILT_ANGLE_LIMIT_MIN);
        angle = TILT_ANGLE_LIMIT_MIN;
    }
    
    if (angle > TILT_ANGLE_LIMIT_MAX) {
        ESP_LOGW(servoTAG, "Clip (%.2f) to max angle (%.2f)", angle, TILT_ANGLE_LIMIT_MAX);
        angle = TILT_ANGLE_LIMIT_MAX;
    }

    setServoAngle(angle);
}

void tiltDegrees(float degrees) {
    // Calculate target angle by adding relative increment to current angle
    float targetAngle = getCurrentAngle() + degrees;
    tiltToAngle(targetAngle);
}

void setServoAngle(float angle) {
    iot_servo_write_angle(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, angle);
    currentTiltAngle = angle;
}

float getCurrentAngle(void) {
    return currentTiltAngle;
}

// Cleanup
void servoCleanup() {
    iot_servo_deinit(LEDC_LOW_SPEED_MODE);
}

// Test function
int servoTest() {
    printf("Servo Control Test\n");
    servoControl_init();
    
    float angle = 0.0;
    float angle_step = 90.0; // Step 90 degrees at a time
    float dir = 1.0;
    while (1) {
        if (angle >= 180.0) {
            dir = -1.0;
        }
        else if (angle <= 0.0) {
            dir = 1.0;
        }
        angle += dir * angle_step;
        tiltToAngle(angle);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    servoCleanup();
    return 0;
}