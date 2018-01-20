/*
 * GccApplication1.c
 *
 * Created: 03.10.2017 20:57:52
 * Author : patlas
 */ 

#include <avr/io.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <string.h>

#include "chars.h"
#include "ds18b20.h"
//#define _BV(x) (1<<x)

#define STORAGE_SIZE 5
#define EEPROM_ADDR 10

#define PIN_UP 3
#define PIN_SET	2
#define BTDDR DDRD
#define BTPORT PORTD
#define BTPIN PIND

#define CLICK_UP !(BTPIN & _BV(PIN_UP))
#define CLICK_SET !(BTPIN & _BV(PIN_SET))

#define LED_DDR	 DDRA
#define LED_PORT PORTA


#define ALED_DDR  DDRD
#define ALED_PORT PORTD
#define ALED1 5
#define ALED2 4

#define PUMP_DDR DDRC
#define PUMP_PORT PORTC
#define PUMP 6

#define PUMP_INIT() {PUMP_DDR |= _BV(PUMP);}
#define PUMP_ON()   {PUMP_PORT &= ~_BV(PUMP);}
#define PUMP_OFF()  {PUMP_PORT |= _BV(PUMP);}

// TODO - checkout
#define BUTTON_TIMEOUT  7*3
#define BUTTON_FAST BUTTON_TIMEOUT/2 //4 * 3
#define BUTTON_VFAST BUTTON_FAST/2 //7
#define MENU_TIMEOUT 1000 // ok. 11s na fabrycznych fusebitach
#define BLINK_TIMEOUT 4*3
#define MEASURE_TIMEOUT 1000

typedef enum {
	OFFSET = 0,
	TEMP1,
	TEMP2,
	SAVE,
	SHOW_TEMP
}menu_position_t;

typedef struct {
	uint8_t offset_val;
	uint8_t temp1_val;
	uint8_t temp2_val;
	bool save;
	uint8_t actual_temp;
}value_storage_t;

uint8_t value_storage[STORAGE_SIZE] = {0};

volatile bool btn_enter = false;
volatile menu_position_t menu_pos = SHOW_TEMP;
volatile uint8_t value = 0;
volatile uint16_t timer_tick = 0;
volatile uint16_t menu_timeout = 0;
volatile uint16_t blink_screen_cnt = 0;
volatile uint16_t temp_measure_cnt = 0;
volatile uint16_t delay_ms_cnt = 0;

volatile bool state_entered = false;
volatile bool get_measurement = true;

bool blink_dot = false;
bool enable_dot = false;
bool dot_on = false;
bool enable_screen = true;
bool enable_blink_screen = false;

static uint8_t default_vals[] =
{
	0, //OFFSET
	20, //TEMP1
	60, //TEMP2
	0 //SAVE
};

static uint8_t TOP_LIMIT[] =
{
	40,
	80,
	80,
	0
};

static uint8_t BOTTOM_LIMIT[] =
{
	0,
	20,
	20,
	0
};

void eeprom_save_config(void)
{
	for(uint8_t i=0; i<STORAGE_SIZE-1; i++)
	{
		eeprom_update_byte((uint8_t*)(EEPROM_ADDR+i), value_storage[i]);
	}
}

void eeprom_load_config(void)
{
	for(uint8_t i=0; i<STORAGE_SIZE-1; i++)
	{
		value_storage[i] = eeprom_read_byte((uint8_t*)(EEPROM_ADDR+i));
	}
}

//value_storage_t value_storage;
static uint8_t high, low;
void print_led(uint8_t val) 
{
	if(200 > val)
	{
		high = val / 10;
		low = val % 10;
	}
	else
	{
		uint8_t temp_val = val - 200;
		switch(temp_val)
		{
			case OFFSET:
				high = 11;//o
				low = 12; //f
			break;
			
			case TEMP1:
				high = 10; //t
				low = 1; //1
			break;
			
			case TEMP2:
				high = 10; //t
				low = 2; //2
			break;
			
			case SAVE:
				high = 5; //s
				low = 13; //d
			break;
			
			default:
			break;
		}
	}
}


void update_screen(uint8_t screen_nr)
{
	uint8_t screen1, screen2;
	
	screen1 = CHARS[high];
	screen2 = CHARS[low];
	
	if(BLINK_TIMEOUT < blink_screen_cnt)
	{
		blink_screen_cnt = 0;
		enable_screen ^= 1;
		dot_on ^= 1;
	}
	
	// if blink enable then able to cause blinking
	enable_screen |= (!enable_blink_screen);
	
	if(!enable_screen)
	{
		screen1 = 0xFF;
		screen2 = 0xFF;
	}
	
	if(0 == screen_nr)
	{
		ALED_PORT |= (1<<ALED2);
		LED_PORT = screen1;
		ALED_PORT &= ~(1<<ALED1);
	}
	else
	{
		ALED_PORT |= (1<<ALED1);
		LED_PORT = screen2;
		ALED_PORT &= ~(1<<ALED2);
		if(enable_dot)
		{			
			if(blink_dot)	
			{
				if(dot_on)
					LED_PORT ^= (1<<LEDDP);
			}
			else
				LED_PORT &= ~(1<<LEDDP);
		}
	}
}



// TODO - write control mechanism for option values (separate manager)
void state_machine(void *args)
{
	// clear timeout timer printing actual temp
	if(value)
	{
		menu_timeout = 0;
	}
	
	
	// disable state_entered in temp view
	if(SHOW_TEMP == menu_pos)
	{
		//value = 0;
		state_entered = false;
		enable_dot = true;
	}
	else
	{
		enable_dot = false;
	}
	
	if(true == state_entered)
	{
		//eventualny check if value has changed
		// add blinking in edit menu
		enable_blink_screen = true;
	
		
		// ich choosen SD dont react
		if(SAVE == menu_pos)
		{
			state_entered = false;
			// save data to nonvolatile eeprom memory
			eeprom_save_config();
			menu_pos+=1;
		}
		else
		{
			value_storage[menu_pos] += value;
			if(value_storage[menu_pos] > TOP_LIMIT[menu_pos])
			{
				value_storage[menu_pos] = BOTTOM_LIMIT[menu_pos];
			}
			print_led(value_storage[menu_pos]);
		}
	}
	else
	{
		menu_pos += value;
		enable_blink_screen = false;

		if(SHOW_TEMP < menu_pos)
		{
			menu_pos = 0;
		}
		
		if(SHOW_TEMP != menu_pos)
		{
			//value = value_storage[menu_pos]; // load default values
			print_led(menu_pos + 200); //high value means text
		}
		else
		{
			if(true == get_measurement)
			{
				//get ds temp
				if(ds18b20_readTemp_nonblocking((temp_non_blocking_t*) args))
				{
					uint8_t divide = false;
					if(value_storage[SHOW_TEMP] > 0)
						divide = true;
						
					value_storage[SHOW_TEMP] += temp1;
					
					if(divide)
						value_storage[SHOW_TEMP] /= 2; //mean temp value from current and previous measurement
					// show dot in that menu
					// if switch is active than blink dot
					get_measurement = false;
				}
			}
			print_led(value_storage[menu_pos]); //print actual temperature
		}
	}
	
	if(temp_measure_cnt >= MEASURE_TIMEOUT)
	{
		temp_measure_cnt = 0;
		get_measurement = true;
	}
	
	value = 0;
}


void init(void){
	ALED_DDR |= _BV(ALED1) | _BV(ALED2);
	LED_DDR	 |= _BV(LEDA) | _BV(LEDB) | _BV(LEDC) | _BV(LEDD) | _BV(LEDE) | _BV(LEDF) | _BV(LEDG) | _BV(LEDDP);

	BTDDR &= ~_BV(PIN_UP) & ~_BV(PIN_SET);
	BTPORT |= _BV(PIN_UP) | _BV(PIN_SET);
	
	PUMP_INIT();
	
	//GICR = 1<<INT0;// | 1<<INT1;		// Enable INT/10
	//MCUCR = 1<<ISC01;// | 1<<ISC11;	// Trigger INT0/1 on rising edge
	
	TIMSK |= _BV(TOIE0);
	TCCR0 |= _BV(CS00) | _BV(CS01) ; //fosc/64 | _BV(CS00); // fosc/1024
	
	sei();				//Enable Global Interrupt
}


/*
ISR(INT1_vect)
{
	if(CLICK_UP)
	{
		//start timer - for speedup incrementation
	}
	else
	{
		//stop timer
	}
}
*/
uint16_t timeout_count = 0;
uint16_t timeout = BUTTON_TIMEOUT;
void button_manager(void)
{	
	if(CLICK_UP)
	{
		if(timer_tick >= timeout)
		{
			timer_tick = 0;
			value = 1;
			//if(value > 85) {value = 0;}
			timeout_count++;
			
			if(timeout_count > 10)
			{
				timeout = BUTTON_FAST;
			}
			else
			{
				timeout = BUTTON_TIMEOUT;
			}
			
		}		
	}
	else
	{
		timeout = BUTTON_TIMEOUT;
		timeout_count = 0;
	}
	
	if(CLICK_SET)
	{
		if(timer_tick >= timeout)
		{
			timer_tick = 0;
			state_entered ^= 1;
			
			/*if(false == state_entered)
			{
				value = menu_pos;
			}*/
		}
	}
}

/*ISR(INT0_vect)
{
	
};*/

ISR(TIMER0_OVF_vect){
	timer_tick++;
	menu_timeout++;
	blink_screen_cnt++;
	temp_measure_cnt++;
	delay_ms_cnt++;
};


//save to eeprom and read from eeprom
// turn on/off opto out

static void drive_pump(void)
{
	uint8_t temp_with_offs = value_storage[OFFSET] + value_storage[SHOW_TEMP];
	
	if(temp_with_offs > value_storage[TEMP1] && temp_with_offs < value_storage[TEMP2])
	{
		PUMP_ON();
		blink_dot = true;
		
	}
	else
	{
		PUMP_OFF();
		blink_dot = false;
	}
	
}

int main(void)
{
	uint8_t sc_nr = 0;
    // ds read temp remove delay, set timer instead (nonblocking)
	
	temp_non_blocking_t temp_cnt_delay;
	temp_cnt_delay.counter = &delay_ms_cnt;
	temp_cnt_delay.after_reset = false;
	temp_cnt_delay.in_delay = false;
	
	// set limit values
	//memcpy(value_storage, BOTTOM_LIMIT, sizeof(BOTTOM_LIMIT));
		
	init();
	PUMP_OFF();
	eeprom_load_config();
	
	while (true) 
    {
		if(MENU_TIMEOUT <= menu_timeout)
		{
			menu_pos = SHOW_TEMP;
		}
		
		drive_pump();
		button_manager();
		state_machine(&temp_cnt_delay);
		
		update_screen(sc_nr);
		sc_nr ^= 1;
		
    }
}

