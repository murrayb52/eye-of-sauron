#include "gimbalControlSvc.h"

#include <stdio.h>
#include "esp_log.h"
#include "stepperControl.h"
#include "servoControl.h"
#include "commsSvc.h"
#include "freertos/queue.h"
#include "joystick.h"

// -------------------------------------------------------------------------------
// Includes

// -------------------------------------------------------------------------------
// Definitions
#define GIMBAL_UPDATE_PERIOD    (50)           /** Gimbal control update period in milliseconds. */
#define SERVO_REL_MODE_ANGLE_STEP (2.0f)             /** Angle step in degrees for relative incremental mode. */

#define IGNORE_TILT_CHANGE_LOWER_LIMIT (85.0f)
#define IGNORE_TILT_CHANGE_UPPER_LIMIT (95.0f)

// -------------------------------------------------------------------------------
// Type defines

// -------------------------------------------------------------------------------
// Local variables
static const char *TAG = "> gimbalControlSvc";  /** Logging tag for the gimbal control service. */
static bool gimbalControlSvc_useRelativeIncMode = true;  /** Flag to enable relative incremental mode for pan control. */

// -------------------------------------------------------------------------------
// Local function declarations
/** @brief Initialize gimbal control subsystems. */
void gimbalControlSvc_init(bool relativeIncMode);

/** @brief Update gimbal position based on current commands. */
static void gimbalControlUpdate(float panAngle, float tiltAngle);

float convertJoystickTiltToIncrement(float tiltAngle);

// -------------------------------------------------------------------------------
// Global function definitions
// -------------------------------------------------------------------------------
/** @brief Initialize gimbal subsystems (servo and stepper motor). */
void gimbalControlSvc_init(bool relativeIncMode)
{
    ESP_LOGI(TAG, "Initializing gimbal control subsystems...");
    servoControl_init();
    stepperControl_init(relativeIncMode);
    joystick_init();
    ESP_LOGI(TAG, "Gimbal control initialized");
}

/**
 * @brief Main FreeRTOS task for the gimbal control service.
 *
 * This task manages servo and stepper motor control for the gimbal system,
 * processing position updates and communicating with other services.
 * Parameter is unused but matches the FreeRTOS task prototype.
 */
void gimbalControlSvc_mainTask(void *pvParameters) {

    (void) pvParameters;

    gimbalAttitude_t lastAttitude = { .panDeg = 0.0, .tiltDeg = 90.0 };

    while (1)
    {
        //ESP_LOGI(TAG, "Updating gimbal control...");

        if (gimbalAttitudeTxQueue != NULL)
        {
            gimbalAttitude_t sample;
            if (xQueueReceive(gimbalAttitudeTxQueue, &sample, 0) == pdTRUE)
            {
                lastAttitude = sample;
                gimbalControlUpdate(lastAttitude.panDeg, lastAttitude.tiltDeg);
            }
        }

        vTaskDelay(GIMBAL_UPDATE_PERIOD / portTICK_PERIOD_MS);
    }
}


// -------------------------------------------------------------------------------
// Local function definitions
// -------------------------------------------------------------------------------
/** @brief Update gimbal position and handle motor control. */
static void gimbalControlUpdate(float panAngle, float tiltAngle)
{
    //ESP_LOGI(TAG, "Updating gimbal control: pan=%.2f, tilt=%.2f", panAngle, tiltAngle);
    // Send pan command to stepper task via queue (non-blocking)
    if (stepperPanQueue != NULL)
    {
        //ESP_LOGI(TAG, "Queue pan command: %.2f degrees", panAngle);
        xQueueOverwrite(stepperPanQueue, &panAngle);
    }

    // Tilt servo is fast, can call directly
    if (gimbalControlSvc_useRelativeIncMode) {
        float tiltChange = convertJoystickTiltToIncrement(tiltAngle);
        tiltDegrees(tiltChange);
    }
    else {
        tiltToAngle(tiltAngle);
    }
}

void testGimbal(float panAngle, float tiltAngle) {
    tiltAngle += 45.0;
    panAngle += 90.0;
    if (tiltAngle > 180.0) tiltAngle = 0.0;
    if (panAngle > 360.0) panAngle = 0.0;
    gimbalControlUpdate(panAngle, tiltAngle);
    vTaskDelay(GIMBAL_UPDATE_PERIOD / portTICK_PERIOD_MS);

    tiltAngle -= 15.0;
    panAngle -= 60.0;
    if (tiltAngle > 180.0) tiltAngle = 0.0;
    if (panAngle > 360.0) panAngle = 0.0;
    gimbalControlUpdate(panAngle, tiltAngle);
    vTaskDelay(GIMBAL_UPDATE_PERIOD / portTICK_PERIOD_MS);
}

float convertJoystickTiltToIncrement(float joystickAngle) {
    float tiltChange = 0.0f;
    if (joystickAngle < IGNORE_TILT_CHANGE_LOWER_LIMIT) {
        ESP_LOGI(TAG, "Tilt down");
        tiltChange = -SERVO_REL_MODE_ANGLE_STEP;
    }
    else if (joystickAngle > IGNORE_TILT_CHANGE_UPPER_LIMIT) {
        ESP_LOGI(TAG, "Tilt up");
        tiltChange = SERVO_REL_MODE_ANGLE_STEP;
    }
    else {
        tiltChange = 0.0f; // No change, return a value that indicates no movement needed
    }
    return tiltChange;
}