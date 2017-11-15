/*
 * chars.h
 *
 *  Created on: 14 lut 2014
 *      Author: patlas
 */

#ifndef CHARS_H_
#define CHARS_H_

#include <avr/pgmspace.h>

#define LEDA 0
#define LEDB 1
#define LEDC 2
#define LEDD 3
#define LEDE 4
#define LEDF 5
#define LEDG 6
#define LEDDP 7

const uint8_t CHARS[] /*PROGMEM*/ = {
		~(_BV(LEDA) |_BV(LEDB) |_BV(LEDC) |_BV(LEDD) |_BV(LEDE) |_BV(LEDF)),
		~(_BV(LEDB) |_BV(LEDC)),
		~(_BV(LEDA) |_BV(LEDB) |_BV(LEDD) |_BV(LEDE) |_BV(LEDG)),
		~(_BV(LEDA) |_BV(LEDB) |_BV(LEDC) |_BV(LEDD) |_BV(LEDG)),
		~(_BV(LEDB) |_BV(LEDC) |_BV(LEDF) |_BV(LEDG)),
		~(_BV(LEDA) |_BV(LEDC) |_BV(LEDD) |_BV(LEDF) |_BV(LEDG)),
		~(_BV(LEDA) |_BV(LEDC) |_BV(LEDD) |_BV(LEDE) |_BV(LEDF) |_BV(LEDG)),
		~(_BV(LEDA) |_BV(LEDB) |_BV(LEDC)),
		~(_BV(LEDA) |_BV(LEDB) |_BV(LEDC) |_BV(LEDD) |_BV(LEDE) |_BV(LEDF) |_BV(LEDG)),
		~(_BV(LEDA) |_BV(LEDB) |_BV(LEDC) |_BV(LEDD) |_BV(LEDF) |_BV(LEDG)),


		~(_BV(LEDD) |_BV(LEDE) |_BV(LEDF) |_BV(LEDG)), //t
		~(_BV(LEDC) |_BV(LEDE) |_BV(LEDD) |_BV(LEDG)), //o
		~(_BV(LEDA) |_BV(LEDE) |_BV(LEDF) |_BV(LEDG)), //f
		~(_BV(LEDB) |_BV(LEDC) |_BV(LEDD) |_BV(LEDE) |_BV(LEDG)), //d
		0xFF //wygaszenie wyświetlacza

};



#endif /* CHARS_H_ */
