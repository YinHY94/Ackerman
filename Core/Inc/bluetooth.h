#ifndef __BLUETOOTH_H__
#define __BLUETOOTH_H__
 
#include "main.h" //HAL库文件声明
#include "usart.h"
#include "Control.h"


extern uint8_t bluetooth_data;


void Get_Bluetooth_Data(uint8_t* bluetooth_data,uint16_t lenth);
void Bluetooth_Data_Process(void);

#endif 
 
 