/******************************************************************************
 * Project        : STM8L_Discovery_SX1278_SFM1L
 * File           : task.c
 * Copyright      : 2014 Yosun Singapore Pte Ltd
 ******************************************************************************
  Change History:

    Version 1.0 - Sep 2014
    > Initial revision

******************************************************************************/
#include "stm8l15x.h"
#include "sx1276.h"
#include "stm8l15x_gpio.h"
#include "stm8l15x_usart.h"
#include "stm8l15x_clk.h"

// used with UU16
# define LSB 1
# define MSB 0

// used with UU32 (b0 is least-significant byte)
# define b0 3
# define b1 2
# define b2 1
# define b3 0

typedef unsigned char U8;
typedef unsigned int U16;
typedef unsigned long U32;

typedef signed char S8;
typedef signed int S16;
typedef signed long S32;

typedef enum
{
  PerStart = 0,		
  PerGoOn,
  PerDone//
}PER_PHASE;

typedef union UU16
{
   U16 U16;
   S16 S16;
   U8 U8[2];
   S8 S8[2];
} UU16;

static uint8_t gb_RxData[256];                                         //Receive data buffer
static uint16_t packageSize = 0;
static uint8_t PER_phase = PerStart;
static UU16  PacketNuNow;
static UU16  PacketNuStart;
//static uint8_t PER;
static uint8_t RxPacketCout;
static __root const u8  cPerStart[] = {"--------StartCout PER--------\x0D\x0A"};
//static const u8  cPERDON[] = {"PER DON..."};
static __root const u8  cGET[] = {"Rec_Packet:"};
static __root const u8  cLost[] =  {"    Lost_Packet:"};
static __root const u8  cToatl[] = {"    Toatl_Packet:"};
static __root const u8  cPer[] =  {"    PER="};
static uint8_t D_value[5];

static void send_char_com(unsigned char UtxData)
{ 
  USART1->CR2 |= 0x08;
  while(!(USART1->SR & 0x80));
  USART1->DR = UtxData;         
  while(!(USART1->SR & 0x40));
  USART1->CR2 &= ~0x08;
}

static void Uart_Prints(unsigned char *pd)
{
  while((*pd)!='\0')
  {
    send_char_com(*pd++);
  }
}

static void HexToAscii_AndUartSent(unsigned char Hex)
{
  unsigned char Ascii[3];
  
  Ascii[0]=(Hex/100)+0x30;
  Ascii[1]=((Hex%100)/10)+0x30;
  Ascii[2]=(Hex%10)+0x30;
  
  send_char_com(Ascii[0]);
  send_char_com(Ascii[1]);
  send_char_com(Ascii[2]);
}

static void Float_Division(uint16_t dividend,uint16_t divisor )
{
  uint8_t i;
  
  for(i = 0; i < 5;i++)
  {
    D_value[i] = dividend/divisor;
    dividend = dividend%divisor;
    if(dividend > 0)
    {
      dividend = dividend *10;
    } 
  }
}

static void PER_Proc(void)
{
	uint8_t p_total;
	uint8_t p_lost;
	switch(PER_phase)//SetTx_Parameters
	       			{
	       				case PerStart:
								RxPacketCout=1;
								
								PacketNuStart.U8[MSB]=gb_RxData[0];
								PacketNuStart.U8[LSB]=gb_RxData[1];
								
								PER_phase=PerGoOn;//payload
								Uart_Prints((u8 *)cPerStart);
								break;
						case PerGoOn:
								RxPacketCout++;
								
								PacketNuNow.U8[MSB]=gb_RxData[0];//MSB
								PacketNuNow.U8[LSB]=gb_RxData[1];

								Uart_Prints((u8 *)cGET);
								HexToAscii_AndUartSent(RxPacketCout);
								
								Uart_Prints((u8 *)cToatl);
								p_total=(PacketNuNow.U16-PacketNuStart.U16+1);
								HexToAscii_AndUartSent(p_total);
								
								Uart_Prints((u8 *)cLost);
								p_lost=p_total-RxPacketCout;
								HexToAscii_AndUartSent(p_lost);
								
								Uart_Prints((u8 *)cPer);							
								Float_Division(p_lost,p_total);///O¢Gg!PAOUD_value[5]AiAa¢FG?D_value[0] !Ni!MsI? //PER=(p_lost/p_total);
								send_char_com(D_value[0]+0x30);
								send_char_com(0x2e);//D!Ey¢Gga
								send_char_com(D_value[1]+0x30);
								send_char_com(D_value[2]+0x30);
								send_char_com(D_value[3]+0x30);
								send_char_com(D_value[4]+0x30);
								
								send_char_com(0x0D);//??DD
								send_char_com(0x0A);//??DD

								if(PacketNuNow.U16<PacketNuStart.U16)
									PER_phase=PerStart;
                                                                
								if((PacketNuNow.U16-PacketNuStart.U16)>=99)
									PER_phase=PerStart;
									
								break;
							
						case PerDone:
								break;
					}

}

/*
 * Manages the master operation
 */
void task_exec(void *p_task)
{
  tRadioDriver *radio = p_task;
  
    switch( radio->Process( ) )
    {
      case RF_RX_TIMEOUT:
          break;
      case RF_RX_DONE:
          radio->GetRxPacket( gb_RxData, ( uint16_t * )&packageSize );
        
          GPIO_ToggleBits(GPIOC, GPIO_Pin_7);
          
          PER_Proc();  
          break;
      case RF_TX_DONE:
          radio->StartRx( );
          break;
      default:
          //send_char_com('K');
          break;
    }
}

void task_init(void)
{  
  /* UART init */
  CLK_PeripheralClockConfig(CLK_Peripheral_USART1,ENABLE);
  USART1->CR2 = 0;
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