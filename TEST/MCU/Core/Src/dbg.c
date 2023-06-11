/*
 * dbg.c
 *
 *  Created on: Jun 10, 2023
 *      Author: admin
 */


#include "dbg.h"

extern UART_HandleTypeDef huart1;

char msg_formatted[300];

void print_str(char* msg,uint32_t uint_param)
{
 sprintf(msg_formatted,"%s:%u\n",msg,uint_param);
 HAL_UART_Transmit(&huart1, msg_formatted, strlen(msg_formatted),100);

}



