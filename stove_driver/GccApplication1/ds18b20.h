/*
 * ds18b20.h
 *
 *  Created on: 27-05-2012
 *      Author: patlas
 */

#ifndef DS18B20_H_
#define DS18B20_H_

volatile unsigned char temp1;
volatile unsigned char temp2;

typedef struct {
	volatile uint16_t *counter; // pointer to instantly incremented timer variable
	bool after_reset; // true if after first reset pulse (in one time shot temp read)
	bool in_delay; // true if delay timeout elapsed (in one time shot temp read sequence)
}temp_non_blocking_t;

unsigned char ds18b20_resetPulse(void);
void ds18b20_sendBit(char bit);
unsigned char ds18b20_receiveBit(void);
void ds18b20_sendByte(unsigned char data);
unsigned char ds18b20_receiveByte(void);
void ds18b20_readTemp(void);
bool ds18b20_readTemp_nonblocking(temp_non_blocking_t *ptr);


#endif /* DS18B20_H_ */
