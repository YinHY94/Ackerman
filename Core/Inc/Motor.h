#ifndef _MOTOR_H_
#define _MOTOR_H_

#include <stdint.h>

#define MOTOR_KP 	0
#define MOTOR_KI 	0
#define MOTOR_KD 	0
#define MOTOR_T  	0.01
#define MOTOR_MAX  	159
#define MOTOR_MIN  	-159

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
double Errer[2]; 		// 当前误差和上一时刻误差
int Ret; 				// PID 输出
int16_t Encoder;		//用10ms采样一次的编码器数值表示速度，范围为-80~80
DIR Dir;
uint8_t Num;			//电机的编号（左后电机为0，右后电机为1）
}Motor;


extern Motor Left_Motor,Right_Motor;
extern int16_t target_encoder;	//移动固定长度时编码器的目标值
extern uint8_t move_distance_state;

void Motor_Set_Direction(Motor* motor,DIR dir);
void Get_Encoder(Motor* motor);
void Motor_Set_Speed(Motor* motor,int16_t speed);
void Motor_Speed_Control(Motor* motor);
void Motor_Stop(void);


#endif

