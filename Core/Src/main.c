#include "stm32f4xx.h"
#include <stdint.h>
#include <stdio.h>
#include "gpio.h"
#include "uart.h"
#include "rgb.h"
#include "i2c.h"
#include "lcd.h"

void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
void TIM4_IRQHandler(void);
volatile uint32_t millis = 0;
uint32_t last_adc_time = - 10;
typedef enum {
    SYS_IDLE = 0,
    SYS_ACTIVE = 1
} SystemState;
SystemState sys_state = SYS_ACTIVE;

typedef enum {
    GAS_NONE = 0,        // 0 – Không có khí
    GAS_LOW = 1,             // 1 – Nồng độ thấp
    GAS_HIGH = 2,            // 2 – Nồng độ cao
    GAS_DANGEROUS = 3        // 3 – Nguy hiểm
} GasState;
GasState detect_gas_level(uint32_t adc) {
	    if (adc < 1000) return GAS_NONE;
	    else if (adc < 2000) return GAS_LOW;
	    else if (adc < 3000) return GAS_HIGH;
	    else return GAS_DANGEROUS;
}
static uint32_t last_toggle = 0;
static uint8_t toggle = 0;
void handle_gas_state(GasState state, SystemState sys) {
    static GasState prev_state = -1;
    static SystemState prev = -1;

    if ((state != prev_state) || (sys != prev)) {
        lcd_set_cursor(1, 0);

        if (sys == SYS_IDLE) {
            RGB_SetPWM(0, 255, 0);  // LED xanh lá
            lcd_send_string("Idle ");
            // Tắt hết thiết bị (HIGH = rơ-le ngắt)
            GPIOA->BSRR = (1 << 8) | (1 << 9) | (1 << 10+16);
        } else {
            switch (state) {
                case GAS_NONE:
                    RGB_SetPWM(0, 0, 255);  // Xanh dương
                    lcd_send_string("0     ");
                    GPIOA->BSRR = (1 << 8);        // PA8 HIGH → Quạt TẮT
                    GPIOA->BSRR = (1 << 9);   // PA9 LOW  → Van BẬT
                    GPIOA->BSRR = (1 << (10+16));       // PA10 HIGH → Buzzer TẮT
                    break;

                case GAS_LOW:
                    RGB_SetPWM(255, 255, 0); // Vàng
                    lcd_send_string("1     ");
                    GPIOA->BSRR = (1 << (8 + 16));   // PA8 LOW  → Quạt BẬT
                    GPIOA->BSRR = (1 << (9));        // PA9 HIGH → Van TẮT
                    GPIOA->BSRR = (1 << (10+16));       // PA10 HIGH → Buzzer TẮT
                    break;

                case GAS_HIGH:
                    last_toggle = millis;
                    toggle = 1;
                    RGB_SetPWM(255, 0, 0);
                    lcd_send_string("2     ");
                    GPIOA->BSRR = (1 << (8 + 16)) | (1 << (9 + 16)) | (1 << 10 ); // Tất cả LOW → Bật
                    break;

                case GAS_DANGEROUS:
                    last_toggle = millis;
                    toggle = 1;
                    RGB_SetPWM(255, 0, 0);
                    lcd_send_string("3     ");
                    GPIOA->BSRR = (1 << (8 + 16)) | (1 << (9 + 16)) | (1 << 10 ); // Tất cả LOW → Bật
                    break;
            }
        }

        prev_state = state;
        prev = sys;
    }

    // Nháy LED đỏ nếu cần
    if (sys == SYS_ACTIVE) {
        if (state == GAS_HIGH && millis - last_toggle >= 500) {
            toggle ^= 1;
            RGB_SetPWM(toggle * 255, 0, 0);
            last_toggle = millis;
        }

        if (state == GAS_DANGEROUS && millis - last_toggle >= 125) {
            toggle ^= 1;
            RGB_SetPWM(toggle * 255, 0, 0);
            last_toggle = millis;
        }
    }
}

// ===== MAIN =====
int main(void) {
    SystemCoreClockUpdate();
    USART2_Init();
    GPIO_Init();         // PC1 - ADC
    Buttons_Init();      // SW1, SW2
    ADC_Init();
    I2C1_Init();
    TIM4_Init();
    RGB_PWM_Init();      // PA5, PA6, PA7 dùng PWM cho LED RGB
//  TIM3_Init();         // Nếu dùng millis để nhấp nháy, bật lại
    USART2_SendString("Hello STM32 + I2C Scan\r\n");
    I2C_Scan();
    lcd_init();

    char msg[40];
    uint32_t adc_value;

    while (1) {
    	if (sys_state == SYS_ACTIVE) {
    	    if ((millis - last_adc_time) >=10) {
    	        last_adc_time = millis;
        		lcd_set_cursor(0,0);
        		lcd_send_string("Gas:");
            	lcd_set_cursor(0, 15);
            	lcd_send_string("1");
        	    ADC1->CR2 |= ADC_CR2_SWSTART;
        	    while (!(ADC1->SR & ADC_SR_EOC));
        	    adc_value = ADC1->DR;
        	    // Gửi UART + LCD
        	    sprintf(msg, "Gas: %lu ppm\r\n", adc_value);
        	    USART2_SendString(msg);
        	    sprintf(msg, "%lu", adc_value);
        	    lcd_set_cursor(0, 5);
        	    lcd_send_string("    ");
        	    lcd_set_cursor(0, 5);
        	    lcd_send_string(msg);
        	    lcd_set_cursor(0, 10);
        	    lcd_send_string("ppm");
    	    }
    	    // Máy trạng thái
    	    GasState current = detect_gas_level(adc_value);
    	    handle_gas_state(current,sys_state);
    	} else {
    	    RGB_SetPWM(0, 255, 0); // Xanh lá
    		lcd_set_cursor(0,0);
    		lcd_send_string("Gas:");
    		lcd_set_cursor(0,15);
    		lcd_send_string("0");
    	}
    }
    while (1);
}



void delay(uint32_t ms) {
    uint32_t start = millis;
    while ((millis - start) < ms);
}

void EXTI0_IRQHandler(void) {
    if (EXTI->PR & (1 << 0)) {
        EXTI->PR |= (1 << 0);
        if (sys_state){
        	sys_state = SYS_IDLE;
        }else{
        	sys_state = SYS_ACTIVE;
        }
    }
}


void EXTI1_IRQHandler(void) {
    if (EXTI->PR & (1 << 1)) {
        EXTI->PR |= (1 << 1); // Clear pending bit
        NVIC_SystemReset();
    }
}

void TIM4_IRQHandler(void) {
    if (TIM4->SR & TIM_SR_UIF) {
        TIM4->SR &= ~TIM_SR_UIF;
        millis++;
    }
}

