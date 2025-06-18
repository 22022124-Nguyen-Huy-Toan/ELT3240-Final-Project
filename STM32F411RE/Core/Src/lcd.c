/*
 * lcd.c
 *
 *  Created on: Jun 18, 2025
 *      Author: DELL
 */
#include "stm32f4xx.h"
#include <stdint.h>
#include <stdio.h>
#include "lcd.h"
#include "i2c.h"
#include "main.h"

void lcd_send_cmd(uint8_t cmd) {
    uint8_t high = cmd & 0xF0;
    uint8_t low  = (cmd << 4) & 0xF0;
    uint8_t data_t[4] = {
        high | 0x0C,
        high | 0x08,
        low  | 0x0C,
        low  | 0x08
    };
    for (int i = 0; i < 4; i++) {
        I2C1_WriteByte(LCD_ADDR, data_t[i]);
        delay(1);
    }
}

void lcd_send_data(uint8_t data) {
    uint8_t high = data & 0xF0;
    uint8_t low  = (data << 4) & 0xF0;
    uint8_t data_t[4] = {
        high | 0x0D,
        high | 0x09,
        low  | 0x0D,
        low  | 0x09
    };
    for (int i = 0; i < 4; i++) {
        I2C1_WriteByte(LCD_ADDR, data_t[i]);
        delay(1);
    }
}

void lcd_init(void) {
    delay(100); // Cho LCD đủ thời gian lên nguồn
    lcd_send_cmd(0x33);
    lcd_send_cmd(0x32);
    lcd_send_cmd(0x28);
    lcd_send_cmd(0x0C);
    lcd_send_cmd(0x06);
    lcd_send_cmd(0x01);
    delay(2);
}

void lcd_send_string(char *str) {
    while (*str) {
        lcd_send_data(*str++);
    }
}
void lcd_set_cursor(uint8_t row, uint8_t col) {
    lcd_send_cmd((row == 0 ? 0x80 : 0xC0) + col);
}


