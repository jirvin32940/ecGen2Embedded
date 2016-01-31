/*****************************************************************************
 * serial_id_ds2411.h
 * From EC Gen I project, adapted for EC Gen II 31jan16.
 ******************************************************************************/
 
 
#ifndef _SERIAL_ID_DS2411_H_
#define _SERIAL_ID_DS2411_H_

#include "same70q21.h"

#define ECLAVE_SERIAL_ID0	PIO_PIN_PA15
#define ECLAVE_SERIAL_ID1	PIO_PIN_PA16
#define ECLAVE_SERIAL_ID2	PIO_PIN_PA17
#define ECLAVE_SERIAL_ID3	PIO_PIN_PA18
#define ECLAVE_SERIAL_ID4	PIO_PIN_PA19


void SetSpeed(int standard);
int OWTouchReset(unsigned char idx);
void OWWriteByte(unsigned char idx, int data);
int OWReadByte(unsigned char idx);
void gpio_input(unsigned char idx);
unsigned char crc8_add(unsigned char acc, unsigned char byte);

#define EC_CPU_CLOCK_100MHZ 100000000UL						//TBD 31jan16 what will this wind up being?
#define EC_CPU_CLOCK_FREQ 8000000 //OSC_RC8M_NOMINAL_HZ		//TBD 31jan16 what will this wind up being?

#endif  // _SERIAL_ID_DS2411_H_
