/*
 * main.h
 *
 *  Created on: Dec 1, 2020
 *      Author: jcaf
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <stdint.h>
#include "types.h"
#include "system.h"


struct _main_flag
{
    unsigned systick :1;
    unsigned sw1_toggle:1;
    unsigned sw1_lock:1;
    unsigned freeze_capture_in_display:1;
    unsigned mv1_updated:1;
    unsigned __a:3;

};
extern struct _main_flag main_flag;



#define SYSTICK 1E-3	//1ms
//#define SYSTICK_MS 1	//1ms



#define PORTWxRELAY1		PORTB
#define PORTRxRELAY1		PINB
#define CONFIGIOxRELAY1		DDRB
#define PINxKB_RELAY1		0

#define PORTWxRELAY2		PORTC
#define PORTRxRELAY2		PINC
#define CONFIGIOxRELAY2		DDRC
#define PINxKB_RELAY2		0

#define PORTWxRELAY3		PORTC
#define PORTRxRELAY3		PINC
#define CONFIGIOxRELAY3		DDRC
#define PINxKB_RELAY3		1


#define PORTWxBUZZER		PORTB
#define PORTRxBUZZER		PINB
#define CONFIGIOxBUZZER		DDRB
#define PINxBUZZER		1


#endif /* MAIN_H_ */
