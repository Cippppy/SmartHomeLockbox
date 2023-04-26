/*
 * FR2355_UART.h
 *
 *  Created on: Dec 7, 2022
 *      Author: traffo17
 */



#ifndef FR2355_UART_H_
#define FR2355_UART_H_

#define MCLK_FREQ_MHZ 8                     // MCLK = 8MHz



void uart_Init_9600();          // Does NOT initialize the GPIO Pins
void Software_Trim();
void uart_Print(char *InputString);







#endif /* FR2355_UART_H_ */
