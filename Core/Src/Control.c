#include "Control.h"

Car_Turning_State Ackerman_Turning={
.Kp=0,
.Ki=0,
.Kd=0,
.T=0.05,
};


CAR_STATE Ackerman_State=STOPING;


int16_t target_encoder;	//移动固定长度时编码器的目标值
uint8_t move_distance_state=0; 

void Turning(double angle,DIR direction){
Ackerman_Turning.Target_Yaw=Ackerman_Turning.Yaw+angle;
Ackerman_Turning.Dir=direction;
if(direction==Reverse){
//Motor_Set_Direction(&Left_Motor,direction);
//Motor_Set_Direction(&Right_Motor,direction);
Motor_Set_Speed(&Left_Motor,20*(direction==Forward?1:-1));
Motor_Set_Speed(&Right_Motor,20*(direction==Forward?1:-1));		
}else{
//Motor_Set_Direction(&Left_Motor,direction);
//Motor_Set_Direction(&Right_Motor,direction);
Motor_Set_Speed(&Left_Motor,20*(direction==Forward?1:-1));
Motor_Set_Speed(&Right_Motor,20*(direction==Forward?1:-1));
}	
}

void Turning_Control(void){
Ackerman_Turning.Errer[1]=Ackerman_Turning.Errer[0];
Ackerman_Turning.Errer[0]=Ackerman_Turning.Yaw-Ackerman_Turning.Target_Yaw;
if(fabs(Ackerman_Turning.Errer[0])>10){
if((Ackerman_Turning.Dir==Forward&&Ackerman_Turning.Errer[0]>0)
	||(Ackerman_Turning.Dir==Reverse&&Ackerman_Turning.Errer[0]<0))
Servo_Set_Angle(-33.3);
else
Servo_Set_Angle(40.5);
}else{
// 积分项计算
Ackerman_Turning.Integral += Ackerman_Turning.Errer[0] * Ackerman_Turning.T;
// 微分项计算
double derivative = (Ackerman_Turning.Errer[0] -Ackerman_Turning.Errer[1]) / Ackerman_Turning.T;
// motor 输出计算
Ackerman_Turning.Ret=Ackerman_Turning.Kp*Ackerman_Turning.Errer[0]+Ackerman_Turning.Ki*Ackerman_Turning.Integral+Ackerman_Turning.Kd*derivative;
Servo_Set_Angle(Ackerman_Turning.Ret);
}	
}

void Go_Straight(void){
Servo_Set_Angle(0);
//Motor_Set_Direction(&Left_Motor,Forward);
//Motor_Set_Direction(&Right_Motor,Forward);
Motor_Set_Speed(&Left_Motor,40);
Motor_Set_Speed(&Right_Motor,40);
}

void Back_Up(void){
Servo_Set_Angle(0);
//Motor_Set_Direction(&Left_Motor,Reverse);
//Motor_Set_Direction(&Right_Motor,Reverse);
Motor_Set_Speed(&Left_Motor,40);
Motor_Set_Speed(&Right_Motor,40);

}

void Stop(void){
Motor_Stop();
}

void Line_Tracking(){
float p=1.0f;
Servo_Set_Angle(p*distance_to_center);
}

void Move_Encoder(Motor* motor,int16_t encoder){
move_distance_state=motor->Num;
if(motor->Num==0){
target_encoder=encoder+(int16_t)TIM1->CNT;
while(target_encoder==(int16_t)TIM1->CNT);}
else if(motor->Num==1){
target_encoder=encoder+(int16_t)TIM3->CNT;
while(target_encoder==(int16_t)TIM3->CNT);}
}


void Car_Control(CAR_STATE Ackerman_State){
switch(Ackerman_State){
case TURNING:
	Turning_Control();
	break;
case TRACKING:
	Line_Tracking();
	break;
case STOPING:
	Stop();
	break;
}
}


void Task1(void){
Stop();
HAL_Delay(1000);
Turning(90,Reverse);
//while()


}


	
	
	

