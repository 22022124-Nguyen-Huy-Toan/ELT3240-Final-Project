/*
 * uart.c
 *
 *  Created on: Jun 18, 2025
 *      Author: DELL
 */
#include "stm32f4xx.h"
#include <stdint.h>
#include <stdio.h>
#include "uart.h"

void USART2_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    GPIOA->MODER &= ~(3 << 4);        // PA2
    GPIOA->MODER |=  (2 << 4);        // Alternate Function
    GPIOA->AFR[0] &= ~(0xF << 8);     // Clear AF
    GPIOA->AFR[0] |=  (7 << 8);       // AF7 = USART2

    USART2->BRR = SystemCoreClock / 9600;
    USART2->CR1 |= USART_CR1_TE;
    USART2->CR1 |= USART_CR1_UE;
}

void USART2_SendChar(char c) {
    while (!(USART2->SR & USART_SR_TXE));
    USART2->DR = c;
}

void USART2_SendString(const char *str) {
    while (*str) {
        USART2_SendChar(*str++);
    }
}

