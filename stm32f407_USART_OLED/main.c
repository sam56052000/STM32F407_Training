#include "main.h"


void Delay_1us(uint32_t nCnt_1us)
{
  volatile uint32_t nCnt;

  for (; nCnt_1us != 0; nCnt_1us--)
    for (nCnt = 13; nCnt != 0; nCnt--);
}

void USART3_IRQHandler(void);
void RCC_Configuration(void)
{
    /* --------------------------- System Clocks Configuration -----------------*/

    /* GPIO clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    /* UART clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    
    /* I2C clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);
}
 
/**************************************************************************************/
 

void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /*-------------------------- GPIO Configuration ----------------------------*/

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD ;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* Connect USART pins to AF */
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);  // USART1_TX
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);  // USART1_RX

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1);  // I2C1 SDA
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);  // I2C1 SCL
}


 
/**************************************************************************************/
 
void LED_Initialization(void){

  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD , ENABLE); //LED3/4 GPIO Port

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 |GPIO_Pin_13 | GPIO_Pin_14 |GPIO_Pin_15;  // LED is connected to PG13/PG14
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

}


void USART3_Configuration(void)
{
    USART_InitTypeDef USART_InitStructure;

    /* USARTx configuration ------------------------------------------------------*/
    /* USARTx configured as follow:
     *  - BaudRate = 57600 baud
     *  - Word Length = 8 Bits
     *  - One Stop Bit
     *  - No parity
     *  - Hardware flow control disabled (RTS and CTS signals)
     *  - Receive and transmit enabled
     */
    USART_InitStructure.USART_BaudRate = 57600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);
    USART_Cmd(USART3, ENABLE);

    USART_ClearFlag(USART3, USART_FLAG_TC);

    USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    /* NVIC Initialization */
    NVIC_InitTypeDef NVIC_InitStruct = {
      .NVIC_IRQChannel = USART3_IRQn,
      .NVIC_IRQChannelPreemptionPriority = 0,
      .NVIC_IRQChannelSubPriority = 0,
      .NVIC_IRQChannelCmd = ENABLE
    };
    NVIC_Init(&NVIC_InitStruct);

}

void I2C_Configuration(void)
{
    I2C_InitTypeDef I2C_InitStructure;

    I2C_SoftwareResetCmd(I2C1, ENABLE);
    I2C_SoftwareResetCmd(I2C1, DISABLE);
 
    I2C_DeInit(I2C1);
    /* IOE_I2C configuration */
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x30;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = 400000;

    /* Initialize the I2C peripheral */
    I2C_Init(I2C1, &I2C_InitStructure);

    /* Enable the I2C peripheral */
    I2C_Cmd(I2C1, ENABLE);
}

void i2c_write(uint8_t address, uint8_t registry, uint8_t data)
{
    while(I2C_GetFlagStatus(IOE_I2C, I2C_FLAG_BUSY));

    I2C_GenerateSTART(IOE_I2C, ENABLE);
    while(!I2C_CheckEvent(IOE_I2C, I2C_EVENT_MASTER_MODE_SELECT));

    I2C_Send7bitAddress(IOE_I2C, address, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(IOE_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    I2C_SendData(I2C1, registry);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    I2C_SendData(I2C1, data);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    
    I2C_GenerateSTOP(I2C1, ENABLE);//¹Ø±ÕI2C1×ÜÏß
}

void USART3_puts(char* s)
{
    while(*s) {
        while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
        USART_SendData(USART3, *s);
        s++;
    }
}


void LED3_On(void){

  GPIO_SetBits(GPIOD,GPIO_Pin_13);

}

void LED3_Off(void){

  GPIO_ResetBits(GPIOD,GPIO_Pin_13);

}

void LED5_On(void){

  GPIO_SetBits(GPIOD,GPIO_Pin_14);

}

void LED5_Off(void){

  GPIO_ResetBits(GPIOD,GPIO_Pin_14);

}

void LED3_Toggle(void){


  GPIO_ToggleBits(GPIOD,GPIO_Pin_13);

}

void LED5_Toggle(void){


  GPIO_ToggleBits(GPIOD,GPIO_Pin_14);

}

uint8_t PushButton_Read(void){

    return GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0);

}

/**************************************************************************************/
int main(void)
{
    RCC_Configuration();
    GPIO_Configuration();
    LED_Initialization();
    USART3_Configuration();
    I2C_Configuration();
    
    
    USART3_puts("Hello I'm handsome Kaiti!\r\n");
    USART3_puts("This is for STM32F407I Discovery verify USART1 with USB TTL Cable\r\n");

    ssd1306_Init();
    
    while(1)
    {


 
    //   if(uart1_data=='a'){
      LED3_On();
      Delay_1us(1000000);
      LED3_Off();
      Delay_1us(1000000);
    // }
    //   if(uart1_data=='b'){
    //   LED5_On();
    // Delay_1us(1000000);
    //  LED5_Off();
    //  Delay_1us(1000000);
    // }

    }


}

void USART3_IRQHandler(void)
{
  
  if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
    uart1_data = USART_ReceiveData(USART3);

    USART_SendData(USART3, uart1_data);


  }

}

