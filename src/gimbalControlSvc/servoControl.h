#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <stdbool.h>

#define TILT_ANGLE_LIMIT_MIN (0.0f)
#define TILT_ANGLE_LIMIT_MAX (180.0f)
#define TILT_ANGLE_INIT      (90.0f)

void servoControl_init(void);
void servoCleanup();
int servoTest();
void tiltToAngle(float angle);
void tiltDegrees(float degrees);
float getCurrentAngle(void);

#endif