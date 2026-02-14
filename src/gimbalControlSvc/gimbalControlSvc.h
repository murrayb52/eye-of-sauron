#ifndef GIMBAL_CONTROL_SVC_H
#define GIMBAL_CONTROL_SVC_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// -------------------------------------------------------------------------------
// Includes

// -------------------------------------------------------------------------------
// Defines

// -------------------------------------------------------------------------------
// Type defines

// -------------------------------------------------------------------------------
// Global variables

// -------------------------------------------------------------------------------
// Global function declarations
/**
 * @brief Main FreeRTOS task for the gimbal control service.
 *
 * This task manages servo and stepper motor control for the gimbal system,
 * processing position updates and communicating with other services.
 *
 * @param pvParameters Unused parameter (matches FreeRTOS task prototype)
 */
void gimbalControlSvc_mainTask(void *pvParameters);

/**
 * @brief Initialize gimbal control subsystems.
 */
void gimbalControlSvc_init(bool relativeIncMode);

#endif // GIMBAL_CONTROL_SVC_H
