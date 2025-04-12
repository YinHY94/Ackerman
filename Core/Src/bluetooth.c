#include "bluetooth.h"

void Get_Bluetooth_Data(uint8_t* bluetooth_data,uint16_t lenth){
HAL_UARTEx_ReceiveToIdle_DMA(&huart1,bluetooth_data,lenth);
}

void Bluetooth_Data_Process(void){
switch(bluetooth_data){
case '1':
	Go_Straight();
break;
case '2':
	Back_Up();
break;
case '3':
	Turning(-90,Forward);
break;
case '4':
	Turning(90,Forward);
break;
case '5':
	Stop();
break;
} 


}
