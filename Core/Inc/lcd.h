/*
 * lcd.h
 *
 *  Created on: Jun 18, 2025
 *      Author: DELL
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

void lcd_send_cmd(uint8_t cmd);
void lcd_send_data(uint8_t data);
void lcd_init(void);
void lcd_send_string(char *str);
void lcd_set_cursor(uint8_t row, uint8_t col);

#endif /* INC_LCD_H_ */
