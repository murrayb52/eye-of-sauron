#ifndef JOYSTICK_H
#define JOYSTICK_H

// -------------------------------------------------------------------------------
// Includes
#include <stdbool.h>
#include "commsSvc.h"

// -------------------------------------------------------------------------------
// Defines


// -------------------------------------------------------------------------------
// Type defines


// -------------------------------------------------------------------------------
// Global variables


// -------------------------------------------------------------------------------
// Global function declarations
/**
 * @brief Read joystick and convert to gimbal attitude.
 *
 * @param attitude Output pointer filled with pan/tilt degrees.
 * @return true if read succeeded, false otherwise.
 */
bool joystick_getInputAttitude(gimbalAttitude_t *attitude);

/**
 * @brief Initialise joystick hardware.
 */
void joystick_init(void);

#endif // JOYSTICK_H
