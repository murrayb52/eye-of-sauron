#include "A4988.h"

extern "C" {

static A4988 *motor = nullptr;

void stepper_init(int steps_per_rev, int dir_pin, int step_pin, int enable_pin, int ms1_pin, int ms2_pin, int ms3_pin)
{
    motor = new A4988(steps_per_rev, dir_pin, step_pin, enable_pin, ms1_pin, ms2_pin, ms3_pin);
}

void stepper_begin(float rpm, short microsteps)
{
    if (motor) {
        motor->begin(rpm, microsteps);
        motor->setEnableActiveState(LOW);  // A4988 enable is active LOW
    }
}

void stepper_move(long steps)
{
    if (motor) {
        motor->move(steps);
    }
}

void stepper_rotate(float degrees)
{
    if (motor) {
        motor->rotate(degrees);
    }
}

void stepper_set_speed(int rpm)
{
    if (motor) {
        motor->setRPM(rpm);
    }
}

void stepper_enable(void)
{
    if (motor) {
        motor->enable();
    }
}

void stepper_disable(void)
{
    if (motor) {
        motor->disable();
    }
}

}