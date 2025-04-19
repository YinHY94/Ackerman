#include "host_computer.h"

uint8_t uart1_rx_data[PACKET_SIZE];
uint8_t mode;
uint8_t angle;
char distance_to_center;

void Host_Computer_Get_Data(uint8_t* uart1_rx_data,uint16_t Size){
if(Size==2){
mode=uart1_rx_data[0]>>5;
angle=uart1_rx_data[0]&0x1F;
distance_to_center=uart1_rx_data[1];
}
}

//void Host_Computer_Data_Process(uint8_t mode,int16_t error){

//}

