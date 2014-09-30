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
#include "board.h"
#include "task.h"

// used with UU16
#define LSB 1
#define MSB 0

// used with UU32 (b0 is least-significant byte)
#define b0 3
#define b1 2
#define b2 1
#define b3 0

#define INPUT_BUFFER_SIZE 50

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
//static __root const u8  cPerStart[] = {"--------StartCout PER--------\x0D\x0A"};
//static const u8  cPERDON[] = {"PER DON..."};
//static __root const u8  cGET[] = {"Rec_Packet:"};
//static __root const u8  cLost[] =  {"    Lost_Packet:"};
//static __root const u8  cToatl[] = {"    Toatl_Packet:"};
//static __root const u8  cPer[] =  {"    PER="};
//static uint8_t D_value[5];
static tTaskInstance taskInstance;
static uint8_t total_input_char_number = 0;
static uint8_t input_buffer[INPUT_BUFFER_SIZE];

static void send_char_com(unsigned char UtxData)
{ 
  USART1->CR2 |= 0x08;
  while(!(USART1->SR & 0x80));
  USART1->DR = UtxData;         
  while(!(USART1->SR & 0x40));
  USART1->CR2 &= ~0x08;
}

#if 0
static void Uart_Prints(unsigned char *pd)
{
  while((*pd)!='\0')
  {
    send_char_com(*pd++);
  }
}
#endif

static void Uart_Prints2(unsigned char *pd, uint16_t len)
{
  uint16_t i;
  
  for(i=0;i<len;i++)
  {
    send_char_com(*pd++);
  }
}

#if 0
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
void RxProc(void *p_task)
{
  tRadioDriver *radio = p_task;
  
    switch( radio->Process( ) )
    {
      case RF_RX_TIMEOUT:
          break;
      case RF_RX_DONE:
          radio->GetRxPacket( gb_RxData, ( uint16_t * )&packageSize );
        
          GPIO_ToggleBits(GPIOC, GPIO_Pin_7);
          
          //PER_Proc();
          Uart_Prints2(gb_RxData, packageSize);
          break;
      case RF_TX_DONE:
          radio->StartRx( );
          break;
      default:
          //send_char_com('K');
          break;
    }
}
#endif

tTaskInstance* task_init(void)
{
  uint8_t i;
  
  for(i=0;i<INPUT_BUFFER_SIZE;i++)
  {
    input_buffer[i] = 0;
  }
  
  taskInstance.p_device1 = 0;
  taskInstance.p_data = input_buffer;
  taskInstance.p_dataLen = &total_input_char_number;
  
  return (&taskInstance);
}

static void discard_input_buffer(void)
{
  uint8_t i;
  
  for(i=0;i<INPUT_BUFFER_SIZE;i++)
  {
    input_buffer[i] = 0;
  }
  
  total_input_char_number = 0;
}

void task_exec(tTaskInstance *task)
{
  tRadioDriver *radio;
  
  if(task == 0)
  {
    return;
  }
  
  radio = (tRadioDriver *)task->p_device1;
  if(radio == 0)
  {
    return;
  }
    
  disableInterrupts();
  /* check if the last char is Carriage Return(CR) */
  /* If yes, then decode the message */
  /* If no, then discard the buffer if max buffer size reach */
  /* Otherwise, wait for CR */
  if((total_input_char_number >0) && (input_buffer[total_input_char_number-1] == '\r'))
  {
    //Uart_Prints2(input_buffer, total_input_char_number);
    
    radio->SetTxPacket(input_buffer, total_input_char_number);
    
    discard_input_buffer();
  }
  else if(total_input_char_number == INPUT_BUFFER_SIZE)
  {
      discard_input_buffer();
  }
  enableInterrupts();
  
  switch( radio->Process( ) )
  {
    case RF_RX_TIMEOUT:
        break;
    case RF_RX_DONE:
        radio->GetRxPacket( gb_RxData, ( uint16_t * )&packageSize );
        
        LoRaRX_Indicate();
          
        //PER_Proc();
        Uart_Prints2(gb_RxData, packageSize);
        
        radio->SetTxPacket(gb_RxData, packageSize);
        break;
    case RF_TX_DONE:
        radio->StartRx( );
        break;
    default:
        //send_char_com('K');
        break;
  }
}

void get_input(void)
{
  /* discard the receiving char if buffer data is not handled by task handler */
  if(total_input_char_number < INPUT_BUFFER_SIZE)
  {
    input_buffer[total_input_char_number++] = USART1->DR;
  }
}
