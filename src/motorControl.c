// motorControl.c
#include "stepper_wrapper.h"  // Include the C wrapper
#include <unistd.h>           // For usleep (ESP-IDF compatible delay)

// Motor configuration (adjust to your ESP32 GPIOs and stepper specs)
#define MOTOR_STEPS 200  // Steps per revolution
#define RPM 120          // Initial RPM
#define MICROSTEPS 1     // Microstepping level
#define DIR_PIN 18       // Direction GPIO
#define STEP_PIN 19      // Step GPIO
#define ENABLE_PIN 21    // Enable GPIO
#define MS1_PIN 22       // Microstep 1 GPIO
#define MS2_PIN 23       // Microstep 2 GPIO
#define MS3_PIN 25       // Microstep 3 GPIO

void motorSetup(void) {
    // Initialize the stepper with pins
    stepper_init(MOTOR_STEPS, DIR_PIN, STEP_PIN, ENABLE_PIN, MS1_PIN, MS2_PIN, MS3_PIN);
    
    // Set up RPM and microstepping
    stepper_begin(RPM, MICROSTEPS);
    
    // Enable the motor (holds position)
    stepper_enable();
}

void motorOperate(void) {
    // Example: Rotate 360 degrees (one full revolution)
    stepper_rotate(360.0f);
    usleep(1000000);  // Delay 1 second (microseconds)
    
    // Example: Move a specific number of steps (back to start)
    stepper_move(-MOTOR_STEPS * MICROSTEPS);  // Negative for reverse
    usleep(1000000);
    
    // Disable motor to allow manual movement
    stepper_disable();
    usleep(5000000);  // Delay 5 seconds
    
    // Re-enable
    stepper_enable();
    
    // Change speed dynamically
    stepper_set_speed(60);  // Slow down to 60 RPM
    stepper_rotate(180.0f);  // Rotate 180 degrees at new speed
}