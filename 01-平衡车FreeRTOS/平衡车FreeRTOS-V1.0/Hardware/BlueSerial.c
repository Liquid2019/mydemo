#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include <stdarg.h>

#define BLUE_SERIAL_BUFFER_SIZE			100

uint8_t BlueSerial_Buffer[BLUE_SERIAL_BUFFER_SIZE];
uint16_t BlueSerial_PutIndex = 0;
uint16_t BlueSerial_GetIndex = 0;

char BlueSerial_String[100];
char BlueSerial_StringArray[6][20];

void BlueSerial_IRQHandler(uint8_t RxData);

void BlueSerial_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART2, &USART_InitStructure);
	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd(USART2, ENABLE);
}

void BlueSerial_SendByte(uint8_t Byte)
{
	USART_SendData(USART2, Byte);
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}

void USART2_IRQHandler(void)
{
	if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
	{
		uint8_t RxData = USART_ReceiveData(USART2);
		BlueSerial_IRQHandler(RxData);
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
	}
}

/*BlueSerial*****************************************/

uint8_t BlueSerial_Put(uint8_t Byte)
{
    if ((BlueSerial_PutIndex + 1) % BLUE_SERIAL_BUFFER_SIZE == BlueSerial_GetIndex)
    {
        return 1;
    }
	BlueSerial_Buffer[BlueSerial_PutIndex] = Byte;
	if (BlueSerial_PutIndex >= BLUE_SERIAL_BUFFER_SIZE - 1)
	{
		BlueSerial_PutIndex = 0;
	}
	else
	{
		BlueSerial_PutIndex ++;
	}
	return 0;
}

uint8_t BlueSerial_Get(uint8_t *Byte)
{
	if (BlueSerial_GetIndex == BlueSerial_PutIndex)
	{
		*Byte = 0;
	    return 1;
	}
	*Byte = BlueSerial_Buffer[BlueSerial_GetIndex];
//	BlueSerial_Buffer[BlueSerial_GetIndex] = 0x00;
	if (BlueSerial_GetIndex >= BLUE_SERIAL_BUFFER_SIZE - 1)
	{
		BlueSerial_GetIndex = 0;
	}
	else
	{
		BlueSerial_GetIndex ++;
	}
	return 0;
}

uint16_t BlueSerial_Length(void)
{
	return (BlueSerial_PutIndex + BLUE_SERIAL_BUFFER_SIZE - BlueSerial_GetIndex) % BLUE_SERIAL_BUFFER_SIZE;;
}

uint8_t BlueSerial_Read(uint16_t Index)
{
	return BlueSerial_Buffer[(BlueSerial_GetIndex + Index) % BLUE_SERIAL_BUFFER_SIZE];
}

void BlueSerial_ClearBuffer(void)
{
	uint8_t i;
	for (i = 0; i < BLUE_SERIAL_BUFFER_SIZE; i ++)
	{
		BlueSerial_Buffer[i] = 0;
	}
}

void BlueSerial_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)
	{
		BlueSerial_SendByte(Array[i]);
	}
}

void BlueSerial_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)
	{
		BlueSerial_SendByte(String[i]);
	}
}

void BlueSerial_Printf(char *format, ...)
{
	char String[100];
	va_list arg;
	va_start(arg, format);
	vsprintf(String, format, arg);
	va_end(arg);
	BlueSerial_SendString(String);
}

uint8_t BlueSerial_ReceiveFlag(void)
{
	uint8_t Flag = 0;
	uint16_t Length = BlueSerial_Length();
	for (uint16_t i = 0; i < Length; i ++)
	{
		if (BlueSerial_Read(i) == '[')
		{
			Flag = 1;
		}
		else if (BlueSerial_Read(i) == ']' && Flag == 1)
		{
			return 1;
		}
	}
	return 0;
}

void BlueSerial_Receive(void)
{
	uint16_t p = 0, k = 0;
	uint8_t Flag = 0;
	uint16_t Length = BlueSerial_Length();
	for (uint16_t i = 0; i < Length; i ++)
	{
		uint8_t Byte;
		BlueSerial_Get(&Byte);
		if (Byte == '[')
		{
			Flag = 1;
			p = 0;
		}
		else if (Byte == ']' && Flag == 1)
		{
			BlueSerial_String[p] = '\0';
			break;
		}
		else if (Flag == 1)
		{
			BlueSerial_String[p] = Byte;
			p ++;
		}
	}
	
	p = 0;
	for (uint16_t i = 0; BlueSerial_String[i] != '\0'; i ++)
	{
		if (BlueSerial_String[i] == ',')
		{
			BlueSerial_StringArray[p][k] = '\0';
			p ++;
			k = 0;
		}
		else
		{
			BlueSerial_StringArray[p][k] = BlueSerial_String[i];
			k ++;
		}
	}
	BlueSerial_StringArray[p][k] = '\0';
}

void BlueSerial_IRQHandler(uint8_t RxData)
{
	BlueSerial_Put(RxData);
}
