#ifndef _HOST_COMPUTER_H_
#define _HOST_COMPUTER_H_

#include "usart.h"
#include <ctype.h>
#include "Control.h"

#define PACKET_SIZE 6
#define ERROR_THRESHOLD 100

extern uint8_t uart1_rx_data[6];
extern Car_Turning_State Ackerman_Turning;
extern uint8_t mode;
extern uint8_t angle;
extern char distance_to_center;

void Host_Computer_Get_Data(uint8_t* uart1_rx_data,uint16_t Size);

#endif
