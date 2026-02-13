#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

void servoControl_init();
void servoCleanup();
int servoTest();
void tiltToAngle(float angle);

#endif