#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void stepper_init(int steps_per_rev, int dir_pin, int step_pin, int enable_pin, int ms1_pin, int ms2_pin, int ms3_pin);
void stepper_move(long steps);
void stepper_rotate(float degrees);
void stepper_set_speed(int rpm);
void stepper_enable(void);
void stepper_disable(void);
void stepper_begin(float rpm, short microsteps);

#ifdef __cplusplus
}
#endif