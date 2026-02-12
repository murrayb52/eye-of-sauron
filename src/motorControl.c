// motorControl.c
#include "motorControl.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Pin definitions - matching working configuration
#define PIN_SLEEP     25
#define PIN_STEP      32
#define PIN_DIR       33

// Motor configuration
#define STEPS_PER_REVOLUTION 200          // Steps per revolution
#define START_DELAY_US 2000      // Start slow (2ms between steps)
#define TARGET_DELAY_US 100      // Target speed (100us between steps)
#define ACCEL_STEPS 50           // Number of steps to ramp up/down

static bool current_dir = 0; // Initial direction (0 or 1)

static const char *TAG = "MotorControl";

static void step_motor(int delay_us) {
    gpio_set_level(PIN_STEP, 1);    // motor will always step on rising edge
    esp_rom_delay_us(2);            // minimum pulse width for A4988 is 1 microsecond, so 2us is safe
    gpio_set_level(PIN_STEP, 0);
    esp_rom_delay_us(delay_us);     // delay between steps controls speed
}

static void toggle_direction(void) {
    current_dir = !current_dir;
    ESP_LOGI(TAG, "Change dir: %d", current_dir);
    gpio_set_level(PIN_DIR, current_dir);
}

static void rotate_revolutions(float revolutions, int speed) {
    // Spin one revolution slowly
    ESP_LOGI(TAG, "rotate %f revs at speed %d", revolutions, speed);

    // convert speed
    int inter_pulse_time_us = 1000;
    if (speed <= 1) {
        inter_pulse_time_us = 5000;

    }
    else if (speed <= 2) {
        inter_pulse_time_us = 4000;
    }
    else {
        inter_pulse_time_us = 3000;
    }

    // do rotations
    int total_steps = revolutions * STEPS_PER_REVOLUTION;
    for (int step = 0; step <= total_steps; step++ ) {
        step_motor(inter_pulse_time_us);
        
        // Yield to other tasks every 100 steps to prevent watchdog timeout
        if (step % 100 == 0) {
            vTaskDelay(1);  // Delay 1 tick (~10ms) to let IDLE task run
        }
    }

    /*
    for (int x = 0; x < STEPS_PER_REVOLUTION * revolutions; x++) {
        gpio_set_level(PIN_STEP, 1);
        esp_rom_delay_us(5); // Speed control
        gpio_set_level(PIN_STEP, 0);
        esp_rom_delay_us(5); // Speed control
    }
    */
}

void motorSetup(void) {
    ESP_LOGI(TAG, "Initializing motor control...");
    
    // Configure GPIO pins
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_STEP) | (1ULL << PIN_DIR) | (1ULL << PIN_SLEEP),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    
    // Wake up the driver (SLEEP high = active)
    gpio_set_level(PIN_SLEEP, 1);
    ESP_LOGI(TAG, "Driver awake");
    
    // Set direction
    gpio_set_level(PIN_DIR, current_dir);
    ESP_LOGI(TAG, "Direction set, motor ready");
}

void motorOperate(void) {
    rotate_revolutions(1.0, 1);
    rotate_revolutions(0.5, 2);

    toggle_direction();
    esp_rom_delay_us(1000000);

    rotate_revolutions(0.5, 2);
    
    esp_rom_delay_us(1000000);
    toggle_direction();

    rotate_revolutions(0.75, 3);
    
    esp_rom_delay_us(1000000);
    toggle_direction();
}