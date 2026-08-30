#ifndef __BLUE_SERIAL_H
#define __BLUE_SERIAL_H

#include <string.h>
#include <stdlib.h>

extern char BlueSerial_String[];
extern char BlueSerial_StringArray[][20];


void BlueSerial_Init(void);
void BlueSerial_SendByte(uint8_t Byte);
void BlueSerial_SendArray(uint8_t *Array, uint16_t Length);
void BlueSerial_SendString(char *String);
void BlueSerial_Printf(char *format, ...);
uint8_t BlueSerial_Put(uint8_t Byte);
uint8_t BlueSerial_Get(uint8_t *Byte);
uint16_t BlueSerial_Length(void);
void BlueSerial_ClearBuffer(void);
uint8_t BlueSerial_ReceiveFlag(void);
void BlueSerial_Receive(void);


#endif
