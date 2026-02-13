/**
 * @file stepperControl.c
 * @brief Stepper motor control via A4988 driver for gimbal yaw (pan) movement.
 *
 * Manages GPIO-based stepper motor control including direction, stepping, and
 * angle-based positioning for the gimbal's pan (yaw) axis.
 */

// -------------------------------------------------------------------------------
// Includes
#include "stepperControl.h"

#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


// -------------------------------------------------------------------------------
// Defines

/** @brief GPIO pin connected to A4988 SLEEP signal. */
#define PIN_SLEEP     25
/** @brief GPIO pin connected to A4988 STEP signal. */
#define PIN_STEP      32
/** @brief GPIO pin connected to A4988 DIR signal. */
#define PIN_DIR       33
/** @brief GPIO pin connected to A4988 RESET signal to revert to start position. */
#define RESET_PAN_PIN 12
/** @brief Steps per motor revolution (A4988 in full-step mode). */
#define STEPS_PER_REVOLUTION 200
/** @brief Initial step delay in microseconds (slow start). */
#define START_DELAY_US 4000
/** @brief Target step delay in microseconds (final speed). */
#define TARGET_DELAY_US 1000
/** @brief Number of steps for acceleration ramp. */
#define PAN_ANGLE_MID 0.0
#define PAN_ANGLE_MIN -135.0
#define PAN_ANGLE_MAX 135.0
#define ACCEL_STEPS 50
#define DIR_CLOCKWISE 1
#define DIR_COUNTERCLOCKWISE 0

// -------------------------------------------------------------------------------
// Type defines

// -------------------------------------------------------------------------------
// Local variables

/** @brief Current motor angle in degrees. */
static float currentAngle = 0.0;
/** @brief Current motor direction (0 or 1). */
static bool currentDir = DIR_COUNTERCLOCKWISE;
/** @brief Logging tag for stepper control messages. */
static const char *TAG = "  -- stepperControl";

// -------------------------------------------------------------------------------
// Local function declarations

/**
 * @brief Generate a single step pulse on the motor.
 * @param[in] delayUs Delay in microseconds between step pulses.
 */
static void stepMotor(int delayUs);

/**
 * @brief Toggle motor direction.
 */
void toggleDirection(void);

/**
 * @brief Set motor direction.
 * @param[in] direction Direction to set (0 for counter-clockwise, 1 for clockwise).
 */
void setDirection(int direction);

void panToAngle(float targetAngle);
int convertAngleToSteps(float angle);
static void moveSteps(int steps);

// -------------------------------------------------------------------------------
// Global function definitions
// -------------------------------------------------------------------------------
void stepperControl_init(void) {
    ESP_LOGI(TAG, "Initializing motor control...");
    
    // Configure GPIO pins
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_STEP) | (1ULL << PIN_DIR) | (1ULL << PIN_SLEEP),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    
    // Wake up the driver (SLEEP high = active)
    gpio_set_level(PIN_SLEEP, 1);
    ESP_LOGI(TAG, "Driver awake");
    
    // Set direction
    setDirection(currentDir);
    ESP_LOGI(TAG, "Direction set, motor ready");
}

// -------------------------------------------------------------------------------
// Local function definitions
// -------------------------------------------------------------------------------

/**
 * @brief Generate a single step pulse on the motor.
 *
 * Produces a 2-microsecond high pulse on the STEP pin to trigger one motor step.
 * A4988 minimum pulse width is 1 microsecond.
 */
static void stepMotor(int delayUs) {
    gpio_set_level(PIN_STEP, 1);    // Motor steps on rising edge
    esp_rom_delay_us(2);            // Minimum pulse width for A4988 is 1µs, 2µs is safe
    gpio_set_level(PIN_STEP, 0);
    esp_rom_delay_us(delayUs);      // Delay between steps controls speed
}

/**
 * @brief Toggle motor direction and update DIR pin.
 */
void toggleDirection(void) {
    currentDir = !currentDir;
    ESP_LOGI(TAG, "Change dir: %d", currentDir);
    setDirection(currentDir);
}

/**
 * @brief Toggle motor direction and update DIR pin.
 */
void setDirection(int direction) {
    currentDir = direction;
    gpio_set_level(PIN_DIR, direction);
}

/**
 * @brief Get current motor direction.
 * @return Current direction (0 for counter-clockwise, 1 for clockwise).
 */
int getDirection() {
    return currentDir;
}


void rotateAngle(float panAngleRel) {
    int pulseSteps = convertAngleToSteps(panAngleRel);
    ESP_LOGI(TAG, "Rotate %f degrees (%d steps)", panAngleRel, pulseSteps);
    moveSteps(pulseSteps);
    currentAngle += panAngleRel;
    if (currentAngle >= 360.0) {
        ESP_LOGW(TAG, "Wrap currentAngle from %f to %f", currentAngle, currentAngle - 360.0);
    }
}

static void moveSteps(int steps) {
    for (int step = 0; step <= steps; step++ ) {
        stepMotor(START_DELAY_US);
        
        // Yield to other tasks every 100 steps to prevent watchdog timeout
        if (step % 100 == 0) {
            //ESP_LOGI(TAG, "vTaskDelay after %d steps", step);
            vTaskDelay(1);  // Delay 1 tick (~10ms) to let IDLE task run
        }
    }
}

void panToAngle(float targetAngle) {
    int dir = DIR_CLOCKWISE;
    if (targetAngle - currentAngle > 180.0) {
        ESP_LOGI(TAG, "Pan clockwise");
    }
    else if (currentAngle - targetAngle > 180.0) {
        dir = DIR_COUNTERCLOCKWISE;
        ESP_LOGI(TAG, "Pan counter-clockwise");
    }

    setDirection(dir);   // move in closest direction to target angle
    float angleDiff = (float)abs((int)(currentAngle - targetAngle));
    rotateAngle(angleDiff);
}

void panToAngleRel(float targetAngleRel) {
    // pan to a target angle relative to the midpoint (0 degrees) in the closest direction
    int dir = DIR_CLOCKWISE;
    if (targetAngleRel < currentAngle) {
        dir = DIR_COUNTERCLOCKWISE;
        ESP_LOGI(TAG, "Pan counter-clockwise");
    }
    else if (targetAngleRel >= currentAngle) {
        ESP_LOGI(TAG, "Pan clockwise");
    }

    setDirection(dir);   // move in closest direction to target angle
    float angleDiff = (float)abs((int)(currentAngle - targetAngleRel));
    rotateAngle(angleDiff);
}


void resetPosition() {
    ESP_LOGI(TAG, "Returning to start position...");
    gpio_set_level(RESET_PAN_PIN, 0); // Assert reset to move to start position
    esp_rom_delay_us(1000000); // Hold reset for 1 second
    gpio_set_level(RESET_PAN_PIN, 1); // Release reset
    currentAngle = 0.0; // Reset current angle tracking
}

int convertAngleToSteps(float angle) {
    return (int)(angle / 360.0 * STEPS_PER_REVOLUTION);
}