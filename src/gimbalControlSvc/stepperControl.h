#ifndef STEPPER_CONTROL_H
#define STEPPER_CONTROL_H

// -------------------------------------------------------------------------------
// Includes
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

// -------------------------------------------------------------------------------
// Defines

// -------------------------------------------------------------------------------
// Type defines

// -------------------------------------------------------------------------------
// Global variables
extern QueueHandle_t stepperPanQueue;

// -------------------------------------------------------------------------------
// Global function declarations

/**
 * @brief Main FreeRTOS task for stepper motor control.
 *
 * @param pvParameters FreeRTOS task parameter (unused).
 */
void stepperControl_mainTask(void *pvParameters);

/**
 * @brief Initialize motor/stepper control subsystem.
 */
void stepperControl_init(void);

/**
 * @brief Operate motor with predefined sequence.
 */
void motorOperate(void);
/**
 * @brief Rotate motor a specified number of revolutions at given speed.
 * @param[in] revolutions Number of full revolutions to rotate (positive or negative).
 * @param[in] speed Speed level (1=slow, 2=medium, 3=fast) for the rotation.
 */
void panToAngle(float angle);
void panToAngleRel(float targetAngleRel);
void setDirection(int clockwise);
void toggleDirection(void);
void moveSteps(int steps);

#ifdef __cplusplus
}
#endif

#endif // STEPPER_CONTROL_H