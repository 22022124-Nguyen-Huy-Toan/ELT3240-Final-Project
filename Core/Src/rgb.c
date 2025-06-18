/*
 * rgb.c
 *
 *  Created on: Jun 18, 2025
 *      Author: DELL
 */


#include "stm32f4xx.h"
#include <stdint.h>
#include <stdio.h>
#include "rgb.h"
void RGB_PWM_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN;

    GPIOA->MODER &= ~((3 << (5*2)) | (3 << (6*2)) | (3 << (7*2)));
    GPIOA->MODER |=  ((2 << (5*2)) | (2 << (6*2)) | (2 << (7*2))); // AF mode

    GPIOA->AFR[0] &= ~((0xF << (5*4)) | (0xF << (6*4)) | (0xF << (7*4)));
    GPIOA->AFR[0] |=  ((1 << (5*4)) | (2 << (6*4)) | (2 << (7*4))); // AF1, AF2, AF2

    // TIM2 - Red
    TIM2->PSC = 84 - 1;
    TIM2->ARR = 255;
    TIM2->CCMR1 |= (6 << 4) | TIM_CCMR1_OC1PE;
    TIM2->CCER |= TIM_CCER_CC1E;
    TIM2->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN;

    // TIM3 - Green & Blue
    TIM3->PSC = 84 - 1;
    TIM3->ARR = 255;
    TIM3->CCMR1 |= (6 << 4) | TIM_CCMR1_OC1PE;       // CH1
    TIM3->CCMR1 |= (6 << 12) | TIM_CCMR1_OC2PE;      // CH2
    TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM3->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN;
}

void RGB_SetPWM(uint8_t r, uint8_t g, uint8_t b) {
    // LED anode chung nên phải đảo
    TIM2->CCR1 = 255 - r;
    TIM3->CCR1 = 255 - g;
    TIM3->CCR2 = 255 - b;
}

