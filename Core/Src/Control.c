#include "Control.h"

Car_Turning_State Ackerman_Turning={
.Kp=0,
.Ki=0,
.Kd=0,
.T=0.05,
};


void Turning(double angle,DIR direction){
Ackerman_Turning.Target_Yaw=Ackerman_Turning.Yaw+angle;
Ackerman_Turning.Dir=direction;
if(direction==Reverse){
//Set_Servo_Angle(-angle);
Set_Motor_Direction(&Left_Motor,direction);
Set_Motor_Direction(&Right_Motor,direction);
Set_Motor_Speed(&Left_Motor,20);
Set_Motor_Speed(&Right_Motor,20);		
}else{
//Set_Servo_Angle(angle);
Set_Motor_Direction(&Left_Motor,direction);
Set_Motor_Direction(&Right_Motor,direction);
Set_Motor_Speed(&Left_Motor,20);
Set_Motor_Speed(&Right_Motor,20);
}	
}

void Turning_Control(void){

Ackerman_Turning.Errer[1]=Ackerman_Turning.Errer[0];
Ackerman_Turning.Errer[0]=Ackerman_Turning.Yaw-Ackerman_Turning.Target_Yaw;
if(fabs(Ackerman_Turning.Errer[0])>10){
if((Ackerman_Turning.Dir==Forward&&Ackerman_Turning.Errer[0]>0)
	||(Ackerman_Turning.Dir==Reverse&&Ackerman_Turning.Errer[0]<0))
Set_Servo_Angle(-33.3);
else
Set_Servo_Angle(40.5);
}else{
// 积分项计算
Ackerman_Turning.Integral += Ackerman_Turning.Errer[0] * Ackerman_Turning.T;
// 微分项计算
double derivative = (Ackerman_Turning.Errer[0] -Ackerman_Turning.Errer[1]) / Ackerman_Turning.T;
// motor 输出计算
Ackerman_Turning.Ret=Ackerman_Turning.Kp*Ackerman_Turning.Errer[0]+Ackerman_Turning.Ki*Ackerman_Turning.Integral+Ackerman_Turning.Kd*derivative;
Set_Servo_Angle(Ackerman_Turning.Ret);
}	
}

void Go_Straight(void){
Set_Servo_Angle(0);
Set_Motor_Direction(&Left_Motor,Forward);
Set_Motor_Direction(&Right_Motor,Forward);
Set_Motor_Speed(&Left_Motor,40);
Set_Motor_Speed(&Right_Motor,40);
}

void Back_Up(void){
Set_Servo_Angle(0);
Set_Motor_Direction(&Left_Motor,Reverse);
Set_Motor_Direction(&Right_Motor,Reverse);
Set_Motor_Speed(&Left_Motor,40);
Set_Motor_Speed(&Right_Motor,40);

}

void Stop(void){
Motor_Stop();
}

