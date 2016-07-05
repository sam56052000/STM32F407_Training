#include "main.h"
#include "stdio.h"
#include "math.h"


static inline void Delay_1us(uint32_t nCnt_1us)
{
  volatile uint32_t nCnt;

  for (; nCnt_1us != 0; nCnt_1us--)
    for (nCnt = 13; nCnt != 0; nCnt--);
}

void RCC_Configuration(void)
{
      /* --------------------------- System Clocks Configuration -----------------*/
      /* GPIOA,GPIOC,USART3 clock enable */
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
      RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
}
 
/**************************************************************************************/
 
void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /*-------------------------- GPIO Configuration for Push Button ----------------------------*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD ;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    /*-------------------------- GPIO Configuration ----------------------------*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* Connect USART pins to AF */
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);   // USART1_TX
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);   // USART1_RX
}

/**************************************************************************************/
 
void LED_Initialization(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD , ENABLE);

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 |GPIO_Pin_13 | GPIO_Pin_14 |GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}

void ADC_Initialization(void)
{
  ADC_InitTypeDef ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
/* Enable ADC3, DMA2 and GPIO clocks ****************************************/
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
  
/* Configure ADC3 Channel7 pin as analog input ******************************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOF, &GPIO_InitStructure);
//ADC1
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_7;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
//ADC2
  
/* ADC Common Init **********************************************************/
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent; // No external trigger is connected
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2; // ADC clock prescaler
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; // No DMA (polling mode)
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);
/* ADC3 Init ****************************************************************/
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; // Resolution 12 bits
  ADC_InitStructure.ADC_ScanConvMode = DISABLE; // Use single channel
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; // Continue conversion
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; // Data bits shifted to right hand side (Low)
  ADC_InitStructure.ADC_NbrOfConversion = 1; // Convert only once
  ADC_Init(ADC3, &ADC_InitStructure);
  ADC_Init(ADC2, &ADC_InitStructure);
  ADC_Init(ADC1, &ADC_InitStructure);
  
/* ADC3 regular channel7 configuration *************************************/
  ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 1, ADC_SampleTime_3Cycles);
  ADC_RegularChannelConfig(ADC2, ADC_Channel_5, 1, ADC_SampleTime_3Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 1, ADC_SampleTime_3Cycles);
  
/* Enable ADC3 */
  ADC_Cmd(ADC3, ENABLE);
  ADC_Cmd(ADC2, ENABLE);
  ADC_Cmd(ADC1, ENABLE);
  
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

void PWM_Initialization(void)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE , ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC , ENABLE); 

  /* -- GPIO Configuration ---------------------------------------------------- */
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);

  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_11 ;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;

  GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_6 ;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;

  GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* -- Timer Configuration --------------------------------------------------- */
  TIM_DeInit(TIM1);
  TIM_DeInit(TIM8);

  TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
  TIM_TimeBaseStruct.TIM_Period = (uint32_t)(20000 - 1);  // 1M/20000=50Hz
  TIM_TimeBaseStruct.TIM_Prescaler = (uint16_t)(168 - 1); // 180MHz/180=1MHz
  TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;    // No division, so 180MHz
  TIM_TimeBaseStruct.TIM_RepetitionCounter = 0;           // Not used
  TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStruct);
  TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStruct);

  TIM_OCInitTypeDef TIM_OCInitStruct;
  TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;               //PWM Edge mode
  TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;   //20000*0.05=1000
  TIM_OCInitStruct.TIM_Pulse = 1000-1;
  TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;        // Output polarity High
  TIM_OCInitStruct.TIM_OCNPolarity = TIM_OCNPolarity_High;      // Complementary output polarity :Not used
  TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Reset;     // No output polarity : reset (low)
  TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCIdleState_Reset;    // Complementary idle output : reset (not used)

  TIM_OC2Init(TIM1, &TIM_OCInitStruct);
  TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
  TIM_OC1Init(TIM8, &TIM_OCInitStruct);
  TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM1, ENABLE);       //Put ARR value into register
  TIM_Cmd(TIM1, ENABLE);                    // Enable Timer 1
  TIM_CtrlPWMOutputs(TIM1, ENABLE);         // Enable output (To GPIO)

  TIM_ARRPreloadConfig(TIM8, ENABLE);       //Put ARR value into register
  TIM_Cmd(TIM8, ENABLE);                    // Enable Timer 1
  TIM_CtrlPWMOutputs(TIM8, ENABLE);         // Enable output (To GPIO)
}

// void TIM1_Initialization(void)
// {
//   GPIO_InitTypeDef GPIO_InitStructure;
//   NVIC_InitTypeDef NVIC_InitStructure;
//   //TIM_ICInitTypeDef  TIM_ICInitStructure;
//   TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
//   /* TIM2 clock enable */
//   RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

//   /* GPIOA clock enable */
//   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

//     /* TIM2  PA5 */  
//   GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8;
//   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//   GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//   GPIO_Init(GPIOA, &GPIO_InitStructure);

//   /* Connect TIM pin to AF5 */
//   GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);

//   /* Enable the TIM2 global Interrupt */
//   NVIC_InitStructure.NVIC_IRQChannel = TIM1_IRQn;
//   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//   NVIC_Init(&NVIC_InitStructure);

//   TIM_DeInit(TIM4);
//   TIM_TimeBaseStruct.TIM_Period = 65535;//65535              
//   TIM_TimeBaseStruct.TIM_Prescaler = 168-1;//50-1          
//   TIM_TimeBaseStruct.TIM_ClockDivision = 0;
//   TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;    // Counter Up
//   TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStruct);

//   TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
//   TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;       //POLARITY!!!!
//   TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;   
//   TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;             //Prescaler
//   TIM_ICInitStructure.TIM_ICFilter = 0x0;

//   TIM_ICInit(TIM1, &TIM_ICInitStructure);

//   /* TIM enable counter */
//   TIM_Cmd(TIM1, ENABLE);
//   /* Enable the CC1 Interrupt Request */
//   TIM_ITConfig(TIM1, TIM_IT_CC1, ENABLE);
  
// }

void TIM4_Initialization(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  //TIM_ICInitTypeDef  TIM_ICInitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
  /* TIM2 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  /* GPIOA clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    /* TIM2  PA5 */  
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Connect TIM pin to AF5 */
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);

  /* Enable the TIM2 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  TIM_DeInit(TIM4);
  TIM_TimeBaseStruct.TIM_Period = 65535;//65535              
  TIM_TimeBaseStruct.TIM_Prescaler = 84-1;//50-1          
  TIM_TimeBaseStruct.TIM_ClockDivision = 0;
  TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;    // Counter Up
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStruct);

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;       //POLARITY!!!!
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;   
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;             //Prescaler
  TIM_ICInitStructure.TIM_ICFilter = 0x0;

  TIM_ICInit(TIM4, &TIM_ICInitStructure);

  /* TIM enable counter */
  TIM_Cmd(TIM4, ENABLE);
  /* Enable the CC1 Interrupt Request */
  TIM_ITConfig(TIM4, TIM_IT_CC1, ENABLE);
  
}

void Timer5_Initialization(void)
{

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the TIM5 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel =  TIM5_IRQn ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitStructure);

  /* -- Timer Configuration --------------------------------------------------- */
  TIM_DeInit(TIM5);
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
  TIM_TimeBaseStruct.TIM_Period = 1000 - 1 ;  // Period 25000 -> 4Hz
  TIM_TimeBaseStruct.TIM_Prescaler = 840 - 1;  // Prescaled by 900 -> = 100000 Hz
  TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1; // Div by one -> 90 MHz
  TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStruct);
  TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
  TIM_ARRPreloadConfig(TIM5, DISABLE);
  TIM_Cmd(TIM5, ENABLE);
}

void SPI_Initialization(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);


  /* SPI configuration -------------------------------------------------------*/
  SPI_I2S_DeInit(SPI3);
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;

  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI3, &SPI_InitStructure);

  /* Enable SPI4  */
  SPI_Cmd(SPI3, ENABLE);
  
  /* Configure GPIO PIN for Chip select */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI3);

  /* Configure GPIO PIN for SPI4 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void USART3_puts(char* s)
{
    while(*s) {
        while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
        USART_SendData(USART3, *s);
        s++;
    }
}


void LED3_On(void)
{
  GPIO_SetBits(GPIOD,GPIO_Pin_13);
}

void LED3_Off(void)
{
  GPIO_ResetBits(GPIOD,GPIO_Pin_13);
}

void LED5_On(void)
{
  GPIO_SetBits(GPIOD,GPIO_Pin_14);
}

void LED5_Off(void)
{
  GPIO_ResetBits(GPIOD,GPIO_Pin_14);
}

void LED3_Toggle(void)
{
  GPIO_ToggleBits(GPIOD,GPIO_Pin_13);
}

void LED5_Toggle(void)
{
  GPIO_ToggleBits(GPIOD,GPIO_Pin_14);
}

uint8_t PushButton_Read(void)
{
    return GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0);
}

//SPI3_read_data function
uint16_t SPI3_read_data(uint8_t send_rigister)
{
  uint16_t spi3data;

  GPIO_ResetBits(GPIOA,GPIO_Pin_4);
  SPI_I2S_SendData(SPI3,send_rigister);
  while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET);
  while (SPI_I2S_GetFlagStatus(SPI3, SPI_FLAG_RXNE) == RESET);
  spi3data=SPI_I2S_ReceiveData(SPI3);

  SPI_I2S_SendData(SPI3,0xFF);
  while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET);
  while (SPI_I2S_GetFlagStatus(SPI3, SPI_FLAG_RXNE) == RESET);
  spi3data=SPI_I2S_ReceiveData(SPI3);
  GPIO_SetBits(GPIOA,GPIO_Pin_4);
  
  return spi3data;
}

//define the asin array,accelemeter value*100+70 mapping to the array is angle
int16_t ang[71]={0, 1, 1, 2, 2, 3, 3, 4, 5, 5, 6
                  , 6, 7, 7, 8, 9, 9,10,10,11,12
                  ,12,13,13,14,14,15,16,16,17,17
                  ,18,19,19,20,20,21,22,22,23,24
                  ,24,25,25,26,27,27,28,29,29,30
                  ,31,31,32,33,33,34,35,35,36,37
                  ,38,38,39,40,41,41,42,43,44,44};

int16_t Acc_trans_to_ang(int16_t data_input)
{ 
  int16_t angle;
  //setting the bound
  if(data_input>70)
  {
    data_input=70;
  }
  if(data_input<-70)
  {
    data_input=-70;
  }

  if(data_input>0)
  {
    angle=ang[data_input];
  }
  else
  {
    data_input=data_input*(-1);
    angle=(-1)*ang[data_input];
  }
  return angle;
}

/**************************************************************************************/
int main(void)
{ 
  //SPI Connect Pin
  //VCC=>5V
  //GND=>GND
  //NCS=>PA4(CS)
  //SCL=>PB3(CLK)
  //ADO=>PB4(MISO)
  //SDA=>PB5(MOSI)

  //mcu init
  RCC_Configuration();
  GPIO_Configuration();
  LED_Initialization();
  USART3_Configuration();
  SPI_Initialization();
  PWM_Initialization();
  TIM4_Initialization();
  Timer5_Initialization();
  ADC_Initialization();
  
  USART3_puts("Hello I'm handsome Kaiti!\r\n");
  USART3_puts("This is for STM32F407I Discovery verify USART1 with USB TTL Cable\r\n");

  //Varible Setting
  uint16_t spidata;
  uint8_t buff_transmit[100];    
  uint8_t i=0 ;
  double dt=0.01f;
  int16_t SPI_ACC_Y,SPI_GYO_X;
  double Ay,Gx;
  double pi=3.1415926f;
  int16_t output=0, outputG=1000,outputR=1000;
  //int8_t j=0;
  double angle=0, accangle=0,setangle=0 ;
  double preerror=0, error=0;
  double integral=0,derivative=0 ;
  double thrintegral=18  , thrderivative=200;
  double PWMhigh=1920, PWMlow=1080;
  double setangleH=30, setangleL=-30;
  double fixangle=-4; // Green positive18

  double kp  =  1.4;
  double ki  =  0.28;
  double kd  =  0.54;
  
  //int16_t accreg=0,gyoreg=0,gyocal=0,reg=0;

  //setting accelemeter register
  GPIO_ResetBits(GPIOA,GPIO_Pin_4);
  Delay_1us(10);
  SPI_I2S_SendData(SPI3,0x1C);
  while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET);
  while (SPI_I2S_GetFlagStatus(SPI3, SPI_FLAG_RXNE) == RESET);
  spidata=SPI_I2S_ReceiveData(SPI3);

  SPI_I2S_SendData(SPI3,0x10);
  while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET);
  while (SPI_I2S_GetFlagStatus(SPI3, SPI_FLAG_RXNE) == RESET);
  spidata=SPI_I2S_ReceiveData(SPI3);
  GPIO_SetBits(GPIOA,GPIO_Pin_4);

  Delay_1us(1000);

  //setting gyrometer register
  GPIO_ResetBits(GPIOA,GPIO_Pin_4);
  Delay_1us(10);
  SPI_I2S_SendData(SPI3,0x1B);
  while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET);
  while (SPI_I2S_GetFlagStatus(SPI3, SPI_FLAG_RXNE) == RESET);
  spidata=SPI_I2S_ReceiveData(SPI3);

  SPI_I2S_SendData(SPI3,0x10);
  while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET);
  while (SPI_I2S_GetFlagStatus(SPI3, SPI_FLAG_RXNE) == RESET);
  spidata=SPI_I2S_ReceiveData(SPI3);
  GPIO_SetBits(GPIOA,GPIO_Pin_4);

  Delay_1us(1000);

  spidata=SPI3_read_data(0x9C);
  sprintf((char *)buff_transmit, "acc_cfg = %d\n",spidata);
  USART3_puts((char *)buff_transmit);

  if(spidata!=16)
  {
    USART3_puts("SPI disconnect!\r\n");
    while(1);
  }

  spidata=SPI3_read_data(0x9B);
  sprintf((char *)buff_transmit, "gyo_cfg = %d\n",spidata);
  USART3_puts((char *)buff_transmit);

  if(spidata!=16)
  {
    USART3_puts("SPI disconnect!\r\n");
    while(1);
  }

   TIM1->CCR2=outputG;
   TIM8->CCR1=outputR;
   Delay_1us(3000000);

   uint16_t adc1,adc2,adc3;

  while(1)
  {
    
    if(freqflag==1)
    {
      freqflag=0;
      LED3_Toggle();
      //Read mpu 6500 accelemetter register
      //Combine ACC Highbit and lowbit
      SPI_ACC_Y=(SPI3_read_data(0xBD)<<8)|SPI3_read_data(0xBE);
      
      //Read mpu 6500 gyroemetter register
      //Combine GYO Highbit and lowbit
      SPI_GYO_X=(SPI3_read_data(0xC3)<<8)|SPI3_read_data(0xC4);
      SPI_GYO_X=SPI_GYO_X+115;

      Ay = (float)SPI_ACC_Y /4096.0f ;
      Gx = (float)SPI_GYO_X /32.8f   ;
      //PWM_output 1=>PE11 TIM1 GREEN,
      //PWM_output 2=>PC6  TIM8 RED,
      //inputcapture =>PA5
      
      if(Ay>0.75)
        {Ay=0.75;}
      if(Ay<-0.75)
        {Ay=-0.75;}
      
      if(timebaseCapture_output<850)
        {
          timebaseCapture_output=1500;
        }
      setangle = -(((timebaseCapture_output-PWMlow)*(setangleL-setangleH)/(PWMlow-PWMhigh))+ setangleL) ;
      accangle = 180*asin(Ay)/pi;
      angle = 0.99*(angle + Gx*dt) + 0.01*accangle ;
      error = angle-setangle -fixangle ;  
      
      integral = integral+error*dt;
      if(integral > thrintegral)
      {integral = thrintegral;}
      if(integral < -thrintegral)
      {integral = -thrintegral;}      
      
      derivative=(error-preerror)/dt;
      if(derivative > thrderivative)
      {derivative = thrderivative;}
      if(derivative < -thrderivative)
      {derivative = -thrderivative;} 
      preerror=error;
      
      output=kp*error+ ki*integral +kd*derivative;


      if(output<-400)
      {output=-400;}
      if(output>400)
      {output=400;}

      outputR = 1400 -output;
      outputG = 1400 +output;
      
      //USART white PC10,green PC11
      //sprintf((char *)buff_transmit, "accelemetterY=%d,gyroemetter=%d,accreg=%d,gyoreg=%d,reg=%d\n",AY,GX,accreg,gyoreg,reg);
      //sprintf((char *)buff_transmit, "Ay=%f Gx=%f \r\n",Ay,Gx);
      //sprintf((char *)buff_transmit, "Ay=%f ,%f ,%d ,%f, %d, %d \r\n",Gx,derivative,output, angle,outputR,outputG );

      // USART3_puts((char *)buff_transmit);


      TIM1->CCR2=outputG;
      TIM8->CCR1=outputR;




      // for (i=0;i<100;i++)
      // {
      //   buff_transmit[i]=0;
      // }

        ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 1, ADC_SampleTime_3Cycles);
        ADC_SoftwareStartConv(ADC3);
        Delay_1us(50);
        adc3 = ADC_GetConversionValue(ADC3);

        ADC_RegularChannelConfig(ADC2, ADC_Channel_5, 1, ADC_SampleTime_3Cycles);
        ADC_SoftwareStartConv(ADC2);
        Delay_1us(50);
        adc2 = ADC_GetConversionValue(ADC2);

        ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 1, ADC_SampleTime_3Cycles);
        ADC_SoftwareStartConv(ADC1);
        Delay_1us(50);
        adc1 = ADC_GetConversionValue(ADC1);
        // sprintf((char *)buff_transmit, "%d\r\n",timebaseCapture_output);
        // sprintf((char *)buff_transmit, "adc1=%d,adc2=%d,adc3=%d\n",adc1,adc2,adc3);
        // sprintf((char *)buff_transmit, "p=%.3f,i=%.3f,d=%.3f,ang=%.1fset=%.1f ,out=%d,\n",kp,ki,kd,angle,setangle,output);
           sprintf((char *)buff_transmit, "P:%.3f I:%.3f D:%.3f angle:%.2f set:%.2f\r\n",kp,ki,kd,angle,setangle);
        USART3_puts((char *)buff_transmit);
        for (i=0;i<100;i++)
        {
          buff_transmit[i]=0;
        }

        // kp=(double)(adc1)*5/4095;
        // ki=(double)(adc2)*2/4095;
        // kd=(double)(adc3)*3/4095;
    }
  }
}

void USART3_IRQHandler(void)
{
  if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
  {
    uart1_data = USART_ReceiveData(USART3);

    USART_SendData(USART3, uart1_data);
  }

}

void TIM5_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
    freqflag=1;
  }
}

void TIM4_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM4, TIM_IT_CC1) == SET) 
  {
    if(hlstatus==1)
    {
      TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);
      timebaseCapture_prev = timebaseCapture_current;
      timebaseCapture_current = TIM_GetCapture1(TIM4);
      TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
      TIM_ICInit(TIM4, &TIM_ICInitStructure);
      hlstatus=0;
    }
    else
    {
      TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);
      timebaseCapture_prev = timebaseCapture_current;
      timebaseCapture_current = TIM_GetCapture1(TIM4);
      TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
      TIM_ICInit(TIM4, &TIM_ICInitStructure);
      hlstatus=1;
      if(timebaseCapture_current > timebaseCapture_prev)
      {

        timebaseCapture_output  = (timebaseCapture_current- timebaseCapture_prev);//*5/18;


      }
      else
      {

        timebaseCapture_output  =  (0xFFFF - timebaseCapture_prev + timebaseCapture_current);//*5/18;
      }
    }
  }
}

// void TIM1_IRQHandler(void)
// {
//   if (TIM_GetITStatus(TIM1, TIM_IT_CC1) == SET) 
//   {
//     if(hlstatus==1)
//     {
//       TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);
//       timebaseCapture_prev = timebaseCapture_current;
//       timebaseCapture_current = TIM_GetCapture1(TIM1);
//       TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
//       TIM_ICInit(TIM4, &TIM_ICInitStructure);
//       hlstatus=0;
//     }
//     else
//     {
//       TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);
//       timebaseCapture_prev = timebaseCapture_current;
//       timebaseCapture_current = TIM_GetCapture1(TIM1);
//       TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
//       TIM_ICInit(TIM, &TIM_ICInitStructure);
//       hlstatus=1;
//       if(timebaseCapture_current > timebaseCapture_prev)
//       {

//         timebaseCapture_output  = (timebaseCapture_current- timebaseCapture_prev);//*5/18;


//       }
//       else
//       {

//         timebaseCapture_output  =  (0xFFFF - timebaseCapture_prev + timebaseCapture_current);//*5/18;
//       }
//     }
//   }
// }

