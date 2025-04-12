#include "Servo.h"

void Set_Servo_Angle(double angle){
unsigned int servo_ccr=(unsigned int)(100*angle/9+SERVO_INIT);
if(servo_ccr>SERVO_MAX)
	servo_ccr=SERVO_MAX;
else if(servo_ccr<SERVO_MIN)
	servo_ccr=SERVO_MIN;
TIM5->CCR1=servo_ccr;
}
