#include "Motor.h"
#include "tim.h"

Motor Left_Motor={
.Kp=MOTOR_KP,
.Ki=MOTOR_KI,
.Kd=MOTOR_KD,
.T=MOTOR_T,	
.Max=MOTOR_MAX,
.Min=MOTOR_MIN,
.Num=0,
};

Motor Right_Motor={
.Kp=MOTOR_KP,
.Ki=MOTOR_KI,
.Kd=MOTOR_KD,
.T=MOTOR_T,	
.Max=MOTOR_MAX,
.Min=MOTOR_MIN,
.Num=1,
};

////电机软启动函数，防止电机启动电流过大
//void Motor_Soft_Start(uint8_t target_speed) {		
//    uint8_t currentSpeed = 0;
//    while (currentSpeed < target_speed) {
//        currentSpeed++;
//        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, currentSpeed);
//        HAL_Delay(5); // 每次增加等待5ms，控制软启动速度
//    }
//}

void Get_Motor_Direction(Motor* motor,DIR direction){
motor->Dir=direction;
}

// 设置电机方向
void Set_Motor_Direction(Motor* motor,DIR dir) {
motor->Dir=dir;	
if (motor->Num==0)
HAL_GPIO_WritePin(L_DIR_GPIO_Port,L_DIR_Pin,motor->Dir ? GPIO_PIN_SET:GPIO_PIN_RESET);
else if (motor->Num==1)
HAL_GPIO_WritePin(R_DIR_GPIO_Port,R_DIR_Pin,motor->Dir ? GPIO_PIN_RESET:GPIO_PIN_SET);
}

//获取编码器数值
void Get_Encoder(Motor* motor){
if (motor->Num==0){
int16_t data=(int16_t)TIM1->CNT;
motor->Encoder=data>0?data:-data;
TIM1->CNT=0;}
else if(motor->Num==1){
int16_t data=(int16_t)TIM3->CNT;
motor->Encoder=data>0?data:-data;
TIM3->CNT=0;}
}

void Set_Motor_Speed(Motor* motor,int16_t speed){
motor->Target=speed;
}

void Motor_Speed_Control(Motor* motor){
//获取当前速度
Get_Encoder(motor);
// 计算当前误差
motor->Errer[1] = motor->Errer[0];
motor->Errer[0] = motor->Encoder - motor->Target;
// 积分项计算
motor->Integral += motor->Errer[0] * motor->T;
// 微分项计算
double derivative = (motor->Errer[0] - motor->Errer[1]) / motor->T;
// motor 输出计算
motor->Ret=motor->Kp*motor->Errer[0]+motor->Ki*motor->Integral+motor->Kd*derivative;
// 限制PWM输出在最大最小范围内
if (motor->Ret > motor->Max) {
motor->Ret = motor->Max;
} else if (motor->Ret < motor->Min) {
motor->Ret = motor->Min;
}
if(motor->Num==0)
TIM2->CCR1=motor->Ret;
else if(motor->Num==1)
TIM2->CCR2=motor->Ret;
}

void Motor_Stop(void){
HAL_TIM_Base_Stop_IT(&htim4);
HAL_TIM_Base_Stop_IT(&htim9);
Set_Motor_Speed(&Left_Motor,0);
Set_Motor_Speed(&Right_Motor,0);
TIM2->CCR1=0;
TIM2->CCR2=0;
}


