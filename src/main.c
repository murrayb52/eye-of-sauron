#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// -------------------------------------------------------------------------------
// Includes

#include "taskMan.h"
#include "gimbalControlSvc.h"
#include "commsSvc.h"
#include "stepperControl.h"

// -------------------------------------------------------------------------------
// Definitions
#define CORE0               0       /**< CPU core 0 identifier for pinned tasks */
// only define CORE1 as 1 if multiple cores exist, otherwise use tskNO_AFFINITY
#define CORE1       ((CONFIG_FREERTOS_NUMBER_OF_CORES > 1) ? 1 : tskNO_AFFINITY) /**< CPU core 1 or tskNO_AFFINITY */
#define TASK_PRIO_3         3       /**< Higher task priority used for main services */
#define TASK_PRIO_2         2       /**< Secondary task priority */

// -------------------------------------------------------------------------------
// Type defines

// -------------------------------------------------------------------------------
// Local variables

// -------------------------------------------------------------------------------
// Local function declarations

// -------------------------------------------------------------------------------
// Global function definitions
// -------------------------------------------------------------------------------
/**
 * @brief Application entry point called by the ESP-IDF runtime.
 *
 * This function initializes services and creates the main FreeRTOS
 * tasks for storage, AP management and auxiliary workloads.
 */
void app_main(void)
{
    gimbalControlSvc_init();
    stepperControl_init();

    vTaskDelay(5000 / portTICK_PERIOD_MS); // Delay to allow gimbal control to initialize before starting comms
    
    xTaskCreatePinnedToCore(gimbalControlSvc_mainTask,  "gimbalControlSvc", 4096, (void*)TASK_ID_GIMBAL_CONTROL,    TASK_PRIO_3, NULL, CORE0);
    xTaskCreatePinnedToCore(stepperControl_mainTask,    "stepperControl",   4096, (void*)TASK_ID_STEPPER_CONTROL,   TASK_PRIO_3, NULL, CORE1);
    xTaskCreatePinnedToCore(commsSvc_mainTask,          "commsSvc",         8192, (void*)TASK_ID_COMMS,             TASK_PRIO_3, NULL, tskNO_AFFINITY);
    //xTaskCreatePinnedToCore(accelSvc_mainTask,          "accelSvc",         4096, (void*)TASK_ID_ACCEL,             TASK_PRIO_3, NULL, tskNO_AFFINITY);
}