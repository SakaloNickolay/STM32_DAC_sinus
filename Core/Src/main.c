/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include <math.h>

#define _USE_MATH_DEFINES
#define M_PI                    (double)3.14159265358979323846  /*PI*/
#define DAC_PERIPH              (uint32_t)0x40007408
#define SIGNAL_BUFF_LENGTH      200
#define APB1_FREQ               90000000  
#define TRIGER_FREQ             (APB1_FREQ/9000)
#define PRESC                   0
#define DAC_MIN_RES             0.00
#define DAC_MAX_RES             4095.00
#define SIN_MIN_RES             -1.00
#define SIN_MAX_RES             1.00
#define OFFSET                  2.00*M_PI/(double)SIGNAL_BUFF_LENGTH

unsigned short signal_dac_buff[SIGNAL_BUFF_LENGTH];
double buffer[SIGNAL_BUFF_LENGTH];

void sinus_gen(double*);
unsigned short match_range(double val,double min1, double max1, double min2, double max2);


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);

int main(void)
{


  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  SystemClock_Config();

  
  sin_gen(buffer);
  
  for(int k = 0; k < SIGNAL_BUFF_LENGTH; k++)
  {
    signal_dac_buff[k] = match_range(buffer[k],SIN_MIN_RES,SIN_MAX_RES,DAC_MIN_RES,DAC_MAX_RES);
  }
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DAC_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();

  while (1)
  {

  }

}

void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_5)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_PWR_EnableOverDriveMode();
  LL_RCC_HSE_EnableBypass();
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_4, 180, LL_RCC_PLLP_DIV_2);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(180000000);
  LL_SetSystemCoreClock(180000000);
  LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);
}

static void MX_DAC_Init(void)
{

  LL_DAC_InitTypeDef DAC_InitStruct;
  LL_GPIO_InitTypeDef GPIO_InitStruct;
  LL_DMA_InitTypeDef DMA_InitStruct;
  
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_DAC1);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  DAC_InitStruct.TriggerSource = LL_DAC_TRIG_EXT_TIM6_TRGO;
  DAC_InitStruct.WaveAutoGeneration = LL_DAC_WAVE_AUTO_GENERATION_NONE;
  DAC_InitStruct.OutputBuffer = LL_DAC_OUTPUT_BUFFER_ENABLE;
  LL_DAC_Init(DAC1, LL_DAC_CHANNEL_1, &DAC_InitStruct);
  
  DMA_InitStruct.Channel = LL_DMA_CHANNEL_7;
  DMA_InitStruct.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
  DMA_InitStruct.FIFOMode = LL_DMA_FIFOMODE_DISABLE;
  DMA_InitStruct.FIFOThreshold = LL_DMA_FIFOSTATUS_50_75;
  DMA_InitStruct.MemBurst = LL_DMA_MBURST_SINGLE;
  DMA_InitStruct.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_HALFWORD;
  DMA_InitStruct.MemoryOrM2MDstAddress = (uint32_t)&signal_dac_buff;
  DMA_InitStruct.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
  DMA_InitStruct.Mode = LL_DMA_MODE_CIRCULAR;
  DMA_InitStruct.NbData  = SIGNAL_BUFF_LENGTH;
  DMA_InitStruct.PeriphBurst = LL_DMA_PBURST_SINGLE;
  DMA_InitStruct.PeriphOrM2MSrcAddress = DAC_PERIPH;
  DMA_InitStruct.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_HALFWORD;
  DMA_InitStruct.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
  DMA_InitStruct.Priority = LL_DMA_PRIORITY_HIGH;
  
  LL_DMA_Init(DMA1,LL_DMA_STREAM_5,&DMA_InitStruct);
  LL_DMA_EnableStream(DMA1,LL_DMA_STREAM_5);
  
  LL_DAC_EnableTrigger(DAC1,LL_DAC_CHANNEL_1);
  LL_DAC_Enable(DAC1,LL_DAC_CHANNEL_1);
  LL_DAC_EnableDMAReq(DAC1,LL_DAC_CHANNEL_1);
  
}


static void MX_TIM6_Init(void)
{
  LL_TIM_InitTypeDef TIM_InitStruct;
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);

  TIM_InitStruct.Prescaler = PRESC;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = (uint16_t)TRIGER_FREQ;
  
  LL_TIM_Init(TIM6, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM6);
  LL_TIM_SetTriggerOutput(TIM6, LL_TIM_TRGO_UPDATE);
  LL_TIM_DisableMasterSlaveMode(TIM6);
  LL_TIM_EnableUpdateEvent(TIM6);
  LL_TIM_EnableCounter(TIM6);
  

}

static void MX_USART2_UART_Init(void)
{

  LL_USART_InitTypeDef USART_InitStruct;

  LL_GPIO_InitTypeDef GPIO_InitStruct;

  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  
  /**USART2 GPIO Configuration
  PA2   ------> USART2_TX
  PA3   ------> USART2_RX
  */
  
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2|LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_8;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART2);
  LL_USART_Enable(USART2);
}

static void MX_GPIO_Init(void)
{
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
}

void Error_Handler(void)
{

}
void sinus_gen(double *mass)
{
  int j = 0;
  for(double i = 0.00; i < 2*M_PI; i+=OFFSET)
  {
      *(mass + j) = sin(i);
      j++;
  }
  
}

unsigned short match_range(double val,double min_Range1, double max_Range1, double min_Range2, double max_Range2)
{
    return (unsigned short)((val-min_Range1)/(max_Range1-min_Range1)*(max_Range2 - min_Range2)+min_Range2);
}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{

}

#endif /* USE_FULL_ASSERT */

