/******************************************************************************
 * Project        : STM8L_Discovery_SX1278_SFM1L
 * File           : board.c
 * Copyright      : 2014 Yosun Singapore Pte Ltd
 ******************************************************************************
  Change History:

    Version 1.0 - Sep 2014
    > Initial revision

******************************************************************************/
#include "stm8l15x.h"
#include "stm8l15x_clk.h"
#include "stm8l15x_gpio.h"
#include "stm8l15x_usart.h"

void board_init(void)
{
  /* Internal clock */
  CLK_SYSCLKSourceConfig(CLK_SYSCLKSource_HSI);
  CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_1);
  
  /* UART init */
  CLK_PeripheralClockConfig(CLK_Peripheral_USART1,ENABLE);
  /* Enable receiver interrupt */
  USART1->CR2 = 0x24;
  USART1->SR = 0;
  USART1->CR1 = 0;
  USART1->CR3 = 0;   
  // baud rate 115200
  //UART1_BRR1 = 0x08; 
  //UART1_BRR2 = 0x0B;
  // baud rate 9600
  USART1->BRR1 = 0x68; 
  USART1->BRR2 = 0x03;
  
  /* LD4 LED blue */
  GPIO_Init(GPIOC, GPIO_Pin_7, GPIO_Mode_Out_PP_High_Fast);
  /* LD3 LED green */
  GPIO_Init(GPIOE, GPIO_Pin_7, GPIO_Mode_Out_PP_High_Fast);
}

void LoRaRX_Indicate(void)
{
  GPIO_ToggleBits(GPIOC, GPIO_Pin_7);
}
