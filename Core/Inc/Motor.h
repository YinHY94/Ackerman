#ifndef _MOTOR_H_
#define _MOTOR_H_

#include <stdint.h>

#define MOTOR_KP 	0
#define MOTOR_KI 	0
#define MOTOR_KD 	0
#define MOTOR_T  	0.01
#define MOTOR_MAX  	160
#define MOTOR_MIN  	0

typedef enum{
Forward,
Reverse
}DIR;

typedef struct{
double Kp;
double Ki;
double Kd;
double T;
double Max;
double Min;
uint8_t Target;
double Integral; 		// 积分项
double Errer[2]; 	// 当前误差和上一时刻误差
int Ret; 			// PID 输出
int16_t Encoder;	//用10ms采样一次的编码器数值表示速度，范围为0-80
DIR Dir;
uint8_t Num;		//电机的编号（左后电机为0，右后电机为1）
}Motor;


extern Motor Left_Motor,Right_Motor;

void Set_Motor_Direction(Motor* motor,DIR dir);
void Get_Encoder(Motor* motor);
void Set_Motor_Speed(Motor* motor,int16_t speed);
void Motor_Speed_Control(Motor* motor);
void Motor_Stop(void);


#endif
