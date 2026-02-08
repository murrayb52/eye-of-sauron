#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include "servoControl.c"

void servo_init();
void jump_to_angle(int angle);
void servo_cleanup();
int servoTest();

#endif