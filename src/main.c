#include "motorControl.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// -------------------------------------------------------------------------------
// Definitions
#define CORE0               0       /**< CPU core 0 identifier for pinned tasks */
// only define CORE1 as 1 if multiple cores exist, otherwise use tskNO_AFFINITY
#define CORE1       ((CONFIG_FREERTOS_NUMBER_OF_CORES > 1) ? 1 : tskNO_AFFINITY) /**< CPU core 1 or tskNO_AFFINITY */
#define TASK_PRIO_3         3       /**< Higher task priority used for main services */
#define TASK_PRIO_2         2       /**< Secondary task priority */

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
    //nvStorageSvc_init();

    //xTaskCreatePinnedToCore(nvStorageSvc_mainTask, "nvStorageTasksSvc", 4096, (void*)TASK_ID_NV_STORAGE, TASK_PRIO_3, NULL, tskNO_AFFINITY);
    //xTaskCreatePinnedToCore(apManagerSvc_mainTask, "apManagerSvc", 8192, (void*)TASK_ID_AP_MANAGER, TASK_PRIO_3, NULL, tskNO_AFFINITY);
    //xTaskCreatePinnedToCore(otherTasksSvc_mainTask, "otherTasksSvc", 4096, (void*)TASK_ID_OTHER, TASK_PRIO_3, NULL, tskNO_AFFINITY);

    motorSetup();  // From motorControl.cpp
    
    while (1) {
        motorOperate();  // Run motor operations
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // ESP-IDF delay
    }

}