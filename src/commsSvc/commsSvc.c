#include "commsSvc.h"

// -------------------------------------------------------------------------------
// Includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "joystick.h"

// -------------------------------------------------------------------------------
// Definitions
#define COMMS_SVC_SAMPLE_PERIOD_MS   (25)    /** Comms service sample period in milliseconds. */


// -------------------------------------------------------------------------------
// Type defines


// -------------------------------------------------------------------------------
// Local variables
static const char *TAG = "> commsSvc";
QueueHandle_t gimbalAttitudeTxQueue = NULL;

// -------------------------------------------------------------------------------
// Local function declarations
// -------------------------------------------------------------------------------
static void commsSvc_init(void);


// -------------------------------------------------------------------------------
// Global function definitions
// -------------------------------------------------------------------------------
void commsSvc_mainTask(void *pvParameters)
{
    (void) pvParameters;
    commsSvc_init();

    while (1)
    {
        gimbalAttitude_t gimbalAttitude = {0};

        if (joystick_getInputAttitude(&gimbalAttitude))
        {
            if (gimbalAttitudeTxQueue != NULL)
            {
                xQueueOverwrite(gimbalAttitudeTxQueue, &gimbalAttitude);
            }
        }

        vTaskDelay(COMMS_SVC_SAMPLE_PERIOD_MS / portTICK_PERIOD_MS);
    }
}

// -------------------------------------------------------------------------------
// Local function definitions
// -------------------------------------------------------------------------------
static void commsSvc_init(void)
{
    gimbalAttitudeTxQueue = xQueueCreate(1, sizeof(gimbalAttitude_t));

    if (gimbalAttitudeTxQueue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create gimbalAttitudeTxQueue");
    }

    joystick_init();
}


/** @} */
