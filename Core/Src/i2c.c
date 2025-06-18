/*
 * i2c.c
 *
 *  Created on: Jun 18, 2025
 *      Author: DELL
 */
#include "stm32f4xx.h"
#include <stdint.h>
#include <stdio.h>
#include "i2c.h"
#include "uart.h"
void I2C1_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // Cấu hình PB6 (SCL) & PB9 (SDA) - cả hai là AF4
    GPIOB->MODER &= ~((3 << (6 * 2)) | (3 << (9 * 2)));
    GPIOB->MODER |=  ((2 << (6 * 2)) | (2 << (9 * 2)));

    GPIOB->OTYPER |= (1 << 6) | (1 << 9);  // Open-drain
    GPIOB->PUPDR  &= ~((3 << (6 * 2)) | (3 << (9 * 2))); // No pull-up/down

    GPIOB->AFR[0] &= ~(0xF << (6 * 4)); // Clear AF for PB6
    GPIOB->AFR[0] |=  (4 << (6 * 4));   // AF4 for PB6 (SCL)

    GPIOB->AFR[1] &= ~(0xF << ((9 - 8) * 4)); // Clear AF for PB9
    GPIOB->AFR[1] |=  (4 << ((9 - 8) * 4));   // AF4 for PB9 (SDA)

    // Cấu hình tốc độ I2C 100kHz @16MHz
    I2C1->CR2 = 16;
    I2C1->CCR = 80;
    I2C1->TRISE = 17;
    I2C1->CR1 |= I2C_CR1_PE;
}
// ===== I2C SCAN =====
void I2C_Scan(void) {
    char msg[32];
    USART2_SendString("Scanning I2C bus...\r\n");
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        I2C1->CR1 |= I2C_CR1_START;
        while (!(I2C1->SR1 & I2C_SR1_SB));
        (void)I2C1->SR1;
        I2C1->DR = addr << 1;

        uint32_t timeout = 1000;
        while (!(I2C1->SR1 & (I2C_SR1_ADDR | I2C_SR1_AF)) && --timeout);

        if (I2C1->SR1 & I2C_SR1_ADDR) {
            (void)I2C1->SR1;
            (void)I2C1->SR2;
            I2C1->CR1 |= I2C_CR1_STOP;
            sprintf(msg, "Found device at 0x%02X\r\n", addr);
            USART2_SendString(msg);
        } else {
            I2C1->CR1 |= I2C_CR1_STOP;
            I2C1->SR1 &= ~I2C_SR1_AF;
        }
    }
    USART2_SendString("I2C scan completed.\r\n");
}

void I2C1_WriteByte(uint8_t addr, uint8_t data) {
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));
    (void)I2C1->SR1;
    I2C1->DR = addr;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR1;
    (void)I2C1->SR2;
    while (!(I2C1->SR1 & I2C_SR1_TXE));
    I2C1->DR = data;
    while (!(I2C1->SR1 & I2C_SR1_TXE));
    I2C1->CR1 |= I2C_CR1_STOP;
}
