#ifndef STEPPER_CONTROL_H
#define STEPPER_CONTROL_H

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
void rotateAngle(float panAngleRel);
void setDirection(int clockwise);
void toggleDirection(void);

#endif // STEPPER_CONTROL_H