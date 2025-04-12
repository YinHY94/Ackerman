#include "JY901.h"

#include <string.h>
#include <stdio.h>
#include "JY901.h"
#include "usart.h"

#define RXBUFFER_LEN 11



uint8_t ACCCALSW[5] = {0XFF,0XAA,0X01,0X01,0X00};//进入加速度校准模式
uint8_t SAVACALSW[5]= {0XFF,0XAA,0X00,0X00,0X00};//保存当前配置

JY901_DATA jy901s_data=
{
//.Rx_flag=0,
.Rx_len=0,
.frame_head=0x55
};

//用串口2给JY模块发送指令
void JY901_Send_CMD(uint8_t* cmd,uint16_t lenth)
{
HAL_UART_Transmit(&huart2,cmd,lenth,HAL_MAX_DELAY);
}

void JY901_Init(void){
HAL_Delay(1000);
JY901_Send_CMD(ACCCALSW,5);
HAL_Delay(100);//等待模块内部自动校准好，模块内部会自动计算需要一定的时间
JY901_Send_CMD(SAVACALSW,5);
HAL_Delay(100);//保存当前配置
}

void JY901_Get_Data(void){
if(jy901s_data.Rx_len < RXBUFFER_LEN) return;
if(jy901s_data.RxBuffer[0]!= jy901s_data.frame_head) return;	//如果帧头不对
switch(jy901s_data.RxBuffer[1])
{
	case 0x51:
		memcpy(&jy901s_data.Accel,jy901s_data.RxBuffer+2,6);
	break;
	
	case 0x52:
		memcpy(&jy901s_data.Gyro,jy901s_data.RxBuffer+2,6);
	break;
	
	case 0x53:
		memcpy(&jy901s_data.Angle,jy901s_data.RxBuffer+2,6);
	break;
}
}

