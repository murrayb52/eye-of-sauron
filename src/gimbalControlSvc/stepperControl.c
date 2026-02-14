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
#define START_DELAY_US 1500
/** @brief Target step delay in microseconds (final speed). */
#define TARGET_DELAY_US 1000
/** @brief Number of steps for acceleration ramp. */
#define PAN_ANGLE_MID 0.0
#define PAN_ANGLE_MIN -135.0
#define PAN_ANGLE_MAX 135.0
#define MIN_ANGLE_DIFF ( 1.0 / STEPS_PER_REVOLUTION * 360.0) // Minimum angle difference to trigger a step
#define DIR_CLOCKWISE 1
#define DIR_COUNTERCLOCKWISE (!DIR_CLOCKWISE)
#define PAN_DEADBAND_ANGLE_NEG (float)(-1.0)
#define PAN_DEADBAND_ANGLE_POS (float)(1.0)
#define relIncModeStepSize 10

// -------------------------------------------------------------------------------
// Type defines

// -------------------------------------------------------------------------------
// Local variables

/** @brief Current motor angle in steps.
 * Zero at centre. Negative  is counter-clockwise, positive is clockwise
*/
static int currentPanSteps = 0;
/** @brief Current motor direction (0 or 1). */
static bool currentDir = DIR_COUNTERCLOCKWISE;
/** @brief Logging tag for stepper control messages. */
static const char *TAG = "  -- stepperControl";

static bool gimbalRelIncMode = true; // true = relative incremental mode, false = absolute position mode

/** @brief Queue for receiving pan angle commands. */
QueueHandle_t stepperPanQueue = NULL;

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
void moveSteps(int steps);
float convertStepsToAngle(int steps);
static int convertAngleToSteps(float angle);
void panDeadbandFilter(float* angle);

// -------------------------------------------------------------------------------
// Global function definitions
// -------------------------------------------------------------------------------
void stepperControl_init(bool relativeIncMode) {
    ESP_LOGI(TAG, "Initializing motor control...");
    gimbalRelIncMode = relativeIncMode;
    
    // Create queue for pan commands (length 1 for xQueueOverwrite)
    stepperPanQueue = xQueueCreate(1, sizeof(float));
    if (stepperPanQueue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create stepperPanQueue");
    }
    
    // Configure GPIO pins
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_STEP) | (1ULL << PIN_DIR) | (1ULL << PIN_SLEEP) | (1ULL << RESET_PAN_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    
    // Wake up the driver (SLEEP high = active)
    gpio_set_level(PIN_SLEEP, 1);
    ESP_LOGI(TAG, "Driver awake");
    
    gpio_set_level(RESET_PAN_PIN, 1);

    // Set direction
    setDirection(currentDir);
    ESP_LOGI(TAG, "Direction set, motor ready");
}

void stepperControl_mainTask(void *pvParameters)
{
    (void)pvParameters;
    
    ESP_LOGI(TAG, "Stepper control task started");
    
    while (1)
    {
        static float panAngle = 0.0;
        
        if (stepperPanQueue != NULL)
        {
            if (xQueueReceive(stepperPanQueue, &panAngle, portMAX_DELAY) == pdTRUE)
            {
                //ESP_LOGI(TAG, "Received pan command: %.2f degrees", panAngle);
                if (gimbalRelIncMode) {
                    if (panAngle < 0) {
                        setDirection(DIR_COUNTERCLOCKWISE);
                        moveSteps(relIncModeStepSize);
                    }
                    else if (panAngle > 0) {
                        setDirection(DIR_CLOCKWISE);
                        moveSteps(relIncModeStepSize);
                    }
                    else {
                        // No movement needed for zero angle
                        continue;
                    }
                }
                else {
                    // Absolute position mode
                    panToAngle(panAngle);
                }
            }
            vTaskDelay(10 / portTICK_PERIOD_MS); // Short delay to prevent tight loop if queue is empty
        }
        else
        {
            ESP_LOGE(TAG, "Stepper pan queue is NULL");
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
}

// -------------------------------------------------------------------------------
// Local function definitions
// -------------------------------------------------------------------------------
static int convertAngleToSteps(float angle) {
    return (int)(angle / 360.0 * STEPS_PER_REVOLUTION);
}

float convertStepsToAngle(int steps) {
    return ((float)steps / STEPS_PER_REVOLUTION) * 360.0f;
}

static int mapDegreesToSteps(float angleVector) {
    // Map angle in degrees to steps, assuming 0 degrees is center, negative is counter-clockwise, positive is clockwise
    int targetAngleInSteps = convertAngleToSteps(angleVector);
    return targetAngleInSteps;
}

void panDeadbandFilter(float* angle) {
    if (*angle > PAN_DEADBAND_ANGLE_NEG && *angle < PAN_DEADBAND_ANGLE_POS) {
        //ESP_LOGI(TAG, "Apply deadband: %.2f is below threshold %.2f, setting to 0", *angle, MIN_ANGLE_DIFF);
        *angle = 0.0f;
    }
}

void panHighPassFilter(int* stepsDiffVector) {
    // Simple high-pass filter to prevent small oscillations around target position
    if (abs(*stepsDiffVector) < convertAngleToSteps(MIN_ANGLE_DIFF)) {
        //ESP_LOGI(TAG, "Apply high-pass filter: %d steps is below threshold %d, setting to 0", *stepsDiffVector, convertAngleToSteps(MIN_ANGLE_DIFF));
        *stepsDiffVector = 0;
    }
}

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
    setDirection(currentDir);
}

/**
 * @brief Toggle motor direction and update DIR pin.
 */
void setDirection(int direction) {
    currentDir = direction;
    if (direction == DIR_CLOCKWISE) {
        ESP_LOGI(TAG, "Set direction: CLOCKWISE");
    }
    else {
        ESP_LOGI(TAG, "Set direction: COUNTERCLOCKWISE");
    }
    gpio_set_level(PIN_DIR, direction);
}

/**
 * @brief Get current motor direction.
 * @return Current direction (0 for counter-clockwise, 1 for clockwise).
 */
int getDirection() {
    return currentDir;
}


void moveSteps(int steps) {
    //ESP_LOGI(TAG, "Move %d steps", steps);
    for (int step = 0; step <= steps; step++ ) {
        stepMotor(START_DELAY_US);
        
        // Yield to other tasks every 100 steps to prevent watchdog timeout
        /*
        if (step % 100 == 0) {
            //ESP_LOGI(TAG, "vTaskDelay after %d steps", step);
            vTaskDelay(1);  // Delay 1 tick (~10ms) to let IDLE task run
        }
        */
    }
}

void panToAngle(float targetAngleFromCentre) {
    // pan to a target angle relative to the midpoint (0 degrees) in the closest direction
    panDeadbandFilter(&targetAngleFromCentre);
    int targetPanSteps = mapDegreesToSteps(targetAngleFromCentre);
    int stepsDiffVector = currentPanSteps - targetPanSteps;
    panHighPassFilter(&stepsDiffVector);
    //ESP_LOGI(TAG, "currentPanSteps: %d, targetPanSteps: %d, stepsDiffVector: %d", currentPanSteps, targetPanSteps, stepsDiffVector);

    if (targetPanSteps < currentPanSteps) {
        setDirection(DIR_COUNTERCLOCKWISE);
        moveSteps(abs(stepsDiffVector));
        currentPanSteps -= abs(stepsDiffVector);
        ESP_LOGI(TAG, "[C-CW] Moved to %.2f degrees (steps: %d)", convertStepsToAngle(currentPanSteps), currentPanSteps);
    }
    else if (targetPanSteps > currentPanSteps){
        setDirection(DIR_CLOCKWISE);
        moveSteps(abs(stepsDiffVector));
        currentPanSteps += abs(stepsDiffVector);
        ESP_LOGI(TAG, "[CW] Moved to %.2f degrees (steps: %d)", convertStepsToAngle(currentPanSteps), currentPanSteps);
    }
    else {
        //  no movement needed
    }
}


void resetPosition() {
    ESP_LOGI(TAG, "Returning to start position...");
    gpio_set_level(RESET_PAN_PIN, 0); // Assert reset to move to start position
    esp_rom_delay_us(1000000); // Hold reset for 1 second
    gpio_set_level(RESET_PAN_PIN, 1); // Release reset
    currentPanSteps = 0; // Reset current angle tracking
}