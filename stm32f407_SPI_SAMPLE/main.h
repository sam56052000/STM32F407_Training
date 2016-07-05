
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f4xx.h"
#include <stdio.h>


void RCC_Configuration(void);
void GPIO_Configuration(void);
uint8_t PushButton_Read(void);

void USART3_Configuration(void);
void USART3_IRQHandler(void);
void USART3_puts(char* s);

void SPI_Initialization(void);

void LED_Initialization(void);
void LED3_Toggle(void);
void LED3_On(void);
void LED3_Off(void);
void LED5_Toggle(void);
void LED5_On(void);
void LED5_Off(void);

static inline void Delay_1us(uint32_t);
uint8_t uart1_data;


#endif /* __MAIN_H */


