//
//	serial.c
//

#include "stm32f0xx_hal.h"

extern UART_HandleTypeDef huart2;

int sendchar( int c){

	//UARTS[port]->SR & USART_FLAG_TXE
	while(__HAL_UART_GET_FLAG( &huart2, UART_FLAG_TXE) == RESET); // Wait for Empty
 
	USART2->TDR = c; // Echo Char
	//USART_ClearITPendingBit(USART3, USART_IT_TXE);

	return 0;
}

int  getkey(void){

	int c;

	while(__HAL_UART_GET_FLAG( &huart2, UART_FLAG_RXNE) == RESET); // Wait for Char
 
	c = USART2->TDR; // Collect Char
	//USART_ClearITPendingBit(USART3, USART_IT_RXNE);
	return c;
}

