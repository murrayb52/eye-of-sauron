#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "motorControl.h"

static const char *TAG = "Main";

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(5000));
    ESP_LOGI(TAG, "Starting motor control application...");
    
    // Initialize motor
    motorSetup();
    
    // Main loop: spin 1 revolution every 2 seconds
    while (1) {
        motorOperate();
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}