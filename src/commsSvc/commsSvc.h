#ifndef COMMS_SVC_H
#define COMMS_SVC_H

// -------------------------------------------------------------------------------
// Includes
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"


// -------------------------------------------------------------------------------
// Defines


// -------------------------------------------------------------------------------
// Type defines
/** @brief Gimbal attitude command in degrees. */
typedef struct gimbalAttitude_t
{
    float panDeg;  /**< Pan angle in degrees (-135.0 to 135.0). */
    float tiltDeg; /**< Tilt angle in degrees (45.0 to 135.0). */
} gimbalAttitude_t;


// -------------------------------------------------------------------------------
// Global variables
extern QueueHandle_t gimbalAttitudeTxQueue;


// -------------------------------------------------------------------------------
// Global function declarations
/**
 * @brief Comms Service Main Task.
 *
 * @param pvParameters FreeRTOS task parameter (unused).
 */
void commsSvc_mainTask(void *pvParameters);


#endif // COMMS_SVC_H
