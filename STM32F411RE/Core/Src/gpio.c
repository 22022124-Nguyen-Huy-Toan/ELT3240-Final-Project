/*
 * gpio.c
 *
 *  Created on: Jun 18, 2025
 *      Author: DELL
 */

#include "stm32f4xx.h"
#include <stdint.h>
#include <stdio.h>
#include "gpio.h"
void TIM4_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    TIM4->PSC = 84 - 1;         // 84MHz / 84 = 1MHz
    TIM4->ARR = 1000 - 1;       // 1ms (1kHz)
    TIM4->DIER |= TIM_DIER_UIE;
    TIM4->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM4_IRQn);
}

void Buttons_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // PA0 và PA1 là input + pull-up
    GPIOA->MODER &= ~((3 << 0) | (3 << 2));  // Input mode
    GPIOA->PUPDR &= ~((3 << 0) | (3 << 2));
    GPIOA->PUPDR |=  (1 << 0) | (1 << 2);    // Pull-up

    // Kết nối EXTI0 và EXTI1 tới PA0 và PA1
    SYSCFG->EXTICR[0] &= ~((0xF << 0) | (0xF << 4));

    // Cho phép EXTI0 và EXTI1
    EXTI->IMR |= (1 << 0) | (1 << 1);
    EXTI->FTSR |= (1 << 0) | (1 << 1); // Ngắt sườn xuống

    // Bật NVIC
    NVIC_EnableIRQ(EXTI0_IRQn);
    NVIC_EnableIRQ(EXTI1_IRQn);
}
void GPIO_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;
    GPIOA->MODER &= ~(3 << (5 * 2));
    GPIOA->MODER |= (2 << (5 * 2));
    GPIOA->AFR[0] |= (1 << (5 * 4));
    GPIOC->MODER |= (3 << (1 * 2));
    // PA8 (Quạt), PA9 (Van), PA10 (Buzzer)
    GPIOA->MODER &= ~((3 << (8 * 2)) | (3 << (9 * 2)) | (3 << (10 * 2)));
    GPIOA->MODER |=  ((1 << (8 * 2)) | (1 << (9 * 2)) | (1 << (10 * 2))); // Output
    GPIOA->OTYPER &= ~((1 << 8) | (1 << 9) | (1 << 10)); // Push-pull
    GPIOA->ODR &= ~((1 << 8) | (1 << 9) | (1 << 10)); // Tắt ban đầu

}
void ADC_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    ADC1->CR2 = 0; // Tắt ADC để cấu hình
    ADC1->SQR1 &= ~(0xF << 20); // 1 lần chuyển đổi
    ADC1->SQR3 = 11; // Kênh 11 (PC1)
    ADC1->SMPR1 |= (7 << 3); // Sampling time 480 cycles cho kênh 11 (bit 3-5)
    ADC1->CR2 |= ADC_CR2_ADON;
}

