/*
 * uart.h
 *
 *  Created on: Jun 18, 2025
 *      Author: DELL
 */

#ifndef INC_UART_H_
#define INC_UART_H_

void USART2_SendString(const char *str);
void USART2_Init(void);
void USART2_SendChar(char c);

#endif /* INC_UART_H_ */
