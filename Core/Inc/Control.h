#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "Motor.h"
//#include "JY901.h"
#include "Servo.h"
#include <math.h>
#include "host_computer.h"

#define WHEEL_BASE             143.5f  // 前后轮距离（mm）
#define TRACK_WIDTH            167f // 前轮距离（mm）


typedef struct {
int16_t R_Wheel_Speed;
int16_t L_Wheel_Speed;
DIR Dir;
double Yaw;
double Gyro;
double Target_Yaw;
double Kp;
double Ki;
double Kd;
double T;
double Integral;
double Errer[2]; 	// 当前误差和上一时刻误差
double Ret; 			//输出舵机转向角度
int16_t Distance;
}Car_Turning_State;


typedef enum  {
TURNING,TRACKING,STOPING
}CAR_STATE;


extern CAR_STATE Ackerman_State;
extern Car_Turning_State Ackerman_Turning;
//extern float current_angle[3];
extern int16_t target_encoder;	//移动固定长度时编码器的目标值
extern uint8_t move_distance_state;



void Car_Control(CAR_STATE Ackerman_State);
void Turning(double angle,DIR direction);
void Turning_Control(void);
void Go_Straight(void);
void Back_Up(void);
void Stop(void);


#endif

