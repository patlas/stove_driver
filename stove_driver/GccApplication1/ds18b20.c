#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>
#include "ds18b20.h"

#define pin1wire PIND
#define ddr1wire DDRD
#define input	1
#define set1wire ddr1wire &= ~_BV(input)
#define clear1wire ddr1wire |= _BV(input)
#define TIMEOUT_750MS  60

char temp[8];

unsigned char ds18b20_resetPulse(void)
{
	clear1wire;
	_delay_us(500);
	set1wire;
	_delay_us(500);
	if(pin1wire & _BV(input)) return 1;
	else return 0;
}

void ds18b20_sendBit(char bit)
{
	clear1wire;
	_delay_us(5);
	if(bit) set1wire;
	_delay_us(80);
	set1wire;
}


unsigned char ds18b20_receiveBit(void)
{
	clear1wire;
	_delay_us(2);
	set1wire;
	_delay_us(15);
	if(pin1wire & _BV(input)) return 1;
	else return 0;
}


void ds18b20_sendByte(unsigned char data)
{
	char index;
	for(index=0;index<8;index++) ds18b20_sendBit((data>>index) & 0x01);
	_delay_us(100);
}

unsigned char ds18b20_receiveByte(void)
{
	unsigned char data = 0;
	char index;
	for(index=0; index<8; index++)
	{
		data |= (ds18b20_receiveBit()<<index);
		_delay_us(15);
	}
	return data;
}

void ds18b20_readTemp(void)
{
	unsigned char t1=0;
	unsigned char t2=0;

	if(ds18b20_resetPulse())
	{
		ds18b20_sendByte(0xCC);
		ds18b20_sendByte(0x44);
		_delay_ms(750);
		ds18b20_resetPulse();

		ds18b20_sendByte(0xCC);
		ds18b20_sendByte(0xBE);
		t1 = ds18b20_receiveByte();
		t2 = ds18b20_receiveByte();
		ds18b20_resetPulse();
		temp1=(t2<<4 | t1>>4);// /10;
		//temp2= (((t1&0x0F)*25)/4);  //// aby wyświetlicz to *25 i /4
	}
}




bool ds18b20_readTemp_nonblocking(temp_non_blocking_t *ptr)
{
	unsigned char t1=0;
	unsigned char t2=0;

	if(!ptr->after_reset)
	{
		ptr->after_reset = (bool) ds18b20_resetPulse();
	}
	else
	{
		if (!ptr->in_delay)
		{
			ds18b20_sendByte(0xCC);
			ds18b20_sendByte(0x44);
			ptr->in_delay = true;
		}
		
		//_delay_ms(750);
		
		if(ptr->in_delay && *(ptr->counter) >= TIMEOUT_750MS)
		{
			ds18b20_resetPulse();

			ds18b20_sendByte(0xCC);
			ds18b20_sendByte(0xBE);
			t1 = ds18b20_receiveByte();
			t2 = ds18b20_receiveByte();
			ds18b20_resetPulse();
			temp1=(t2<<4 | t1>>4);// /10;
			
			ptr->after_reset = false;
			*(ptr->counter) = 0;
			ptr->in_delay = false;
			
			return true; // measurement ready
		}

	}
	
	return false;
}
