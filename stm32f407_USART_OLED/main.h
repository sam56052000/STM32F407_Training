
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f4xx.h"
#include "./ssd1306/ssd1306.h"

#define IOE_I2C I2C1

uint8_t uart1_data;
void RCC_Configuration(void);
void GPIO_Configuration(void);
void LED_Initialization(void);
void LED3_Toggle(void);
void LED3_On(void);
void LED3_Off(void);
void LED5_Toggle(void);
void LED5_On(void);
void LED5_Off(void);
void USART3_IRQHandler(void);
void USART3_Configuration(void);
void USART3_puts(char* s);
uint8_t PushButton_Read(void);
void Delay_1us(uint32_t nCnt_1us);

void I2C_Configuration(void);
void i2c_write(uint8_t address, uint8_t registry, uint8_t data);

#endif /* __MAIN_H */


