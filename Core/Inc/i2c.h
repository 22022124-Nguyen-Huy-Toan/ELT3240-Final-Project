/*
 * i2c.h
 *
 *  Created on: Jun 18, 2025
 *      Author: DELL
 */

#ifndef INC_I2C_H_
#define INC_I2C_H_
#define LCD_ADDR (0x27 << 1)
void I2C1_Init(void);
void I2C_Scan(void);
void I2C1_WriteByte(uint8_t addr, uint8_t data);

#endif /* INC_I2C_H_ */
