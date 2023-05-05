/*
 * GPIO_Driver.h
 *
 *  Created on: Jan 6, 2023
 *      Author:
 */

#ifndef GPIO_DRIVER_H_
#define GPIO_DRIVER_H_

void gpioInit(char Port, char Pin, char Direction);
void gpioWrite(char Port, char Pin, char Value);
char gpioRead(char Port, char Pin);

#endif /* GPIO_DRIVER_H_ */
