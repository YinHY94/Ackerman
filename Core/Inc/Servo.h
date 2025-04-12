#ifndef _SERVO_H_
#define _SERVO_H_

#include "tim.h"

#define SERVO_INIT (1500-1)
#define SERVO_MAX (1950-1)
#define SERVO_MIN (1130-1)


void Set_Servo_Angle(double angle);


#endif
