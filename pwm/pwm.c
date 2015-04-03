/*
 * pwm.c
 *
 *  Created on: Oct 13, 2014
 *      Author: jcobb
 */
#include <avr/pgmspace.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include "../util/clock.h"
#include "../util/log.h"
#include "pwm.h"

#define LARGEST_8BIT_NUMBER		255
#define LARGEST_16BIT_NUMBER	65535
#define PWM_DEFAULT				1500

// TIME = PERIOD/FREQUENCY
// 0.000032 = 256/8,000,000

static volatile uint16_t duty_cycle_channel4 = PWM_DEFAULT;
static volatile uint16_t duty_cycle_channel5 = PWM_DEFAULT;

static const char _tag[] PROGMEM = "pwm: ";

void pwm_init()
{

	// setup our terminal callback

	DDRB |= (1<<DDB1);
	DDRB |= (1<<DDB2);


	// Fast PWM overflow on REG A and REG B

	TCCR1A |= _BV(COM1A1);
	TCCR1A |= _BV(COM1B1);
	TCCR1A |= _BV(WGM11);
	TCCR1B |= _BV(WGM13) | _BV(WGM12);
	TCCR1B |= _BV(CS11);
	ICR1 = 19999;
	TIMSK1 |= _BV(TOIE1);

	// Set interrupt enable
	sei();

	OCR1A = duty_cycle_channel5;
	OCR1B = duty_cycle_channel4;
}

void pwm_setval(uint16_t val, uint16_t channel)
{
	if(channel == 4)
	{
		duty_cycle_channel4 = val;
	}
	else if (channel == 5)
	{
		duty_cycle_channel5 = val;
	}
}

ISR(TIMER1_OVF_vect)
{
	OCR1A = duty_cycle_channel5; // PB1
	OCR1B = duty_cycle_channel4; // PB2
}



/*
https://sites.google.com/site/qeewiki/books/avr-guide/common-timer-theory
https://sites.google.com/site/qeewiki/books/avr-guide/timers-on-the-atmega328

// CS BITS
CS02    CS01    CS00     DESCRIPTION
0       0       0        Timer/Counter0 Disabled
0       0       1        No Prescaling
0       1       0        Clock / 8
0       1       1        Clock / 64
1       0       0        Clock / 256
1       0       1        Clock / 1024

CS12     CS11    CS10    DESCRIPTION
0       0       0        Timer/Counter1 Disabled
0       0       1        No Prescaling
0       1       0        Clock / 8
0       1       1        Clock / 64
1       0       0        Clock / 256
1       0       1        Clock / 1024

CS22     CS21    CS20    DESCRIPTION
0       0       0        Timer/Counter2 Disabled
0       0       1        No Prescaling
0       1       0        Clock / 8
0       1       1        Clock / 32
1       0       0        Clock / 64
1       0       1        Clock / 128
1       1       0        Clock / 256
1       1       1        Clock / 1024


// WAVEFORM GENERATOR BITS
        WGM02   WGM01   WGM00    DESCRIPTION            TOP
0       0       0       0        Normal                 0xFF
1       0       0       1        PWM, Phase Corrected   0xFF
2       0       1       0        CTC                    OCR0A
3       0       1       1        Fast PWM               0xFF
4       1       0       0        Reserved               -
5       1       0       1        Fast PWM, Phase Corr   OCR0A
6       1       1       0        Reserved               -
7       1       1       1        Fast PWM               OCR0A

MODE    WGM13   WGM12   WGM11   WGM10    DESCRIPTION                     TOP
0        0       0      0       0        Normal                          0xFFFF
1       0       0       0       1        PWM, Phase Corrected, 8bit      0x00FF
2       0       0       1       0        PWM, Phase Corrected, 9bit      0x01FF
3       0       0       1       1        PWM, Phase Corrected, 10bit     0x03FF
4       0       1       0       0        CTC                             OCR1A
5       0       1       0       1        Fast PWM, 8bit                   0x00FF
6       0       1       1       0        Fast PWM, 9bit                   0x01FF
7       0       1       1       1        Fast PWM, 10bit                  0x03FF
8       1       0       0       0        PWM, Phase and Frequency Corr    ICR1
9       1       0       0       1        PWM, Phase and Frequency Corr    OCR1A
10      1       0       1       0        PWM, Phase Correct               ICR1
11      1       0       1       1        PWM, Phase Correct              OCR1A
12      1       1       0       0        CTC                             ICR1
13      1       1       0       1        RESERVED
14      1       1       1       0        Fast PWM                         ICR1
15      1       1       1       1        Fast PWM                         OCR1A

MODE    WGM21   WGM20    DESCRIPTION              TOP
0       0       0        Normal                  0xFF
1       0       1        PWM Phase Corrected
2       1       0        CTC                      OCR2
3       1       1        Fast PWM



x = Timer Number
        7 bit    6 bit   5 bit   4 bit   3 bit   2 bit   1 bit   0 bit     Description
TCCRxA  COMxA1   COMxA0  COMxB1  COMxB0  -       -       WGMx1   WGMx0     Timer/Counter Control Register x A (x=0,2)

TCCR1B  ICNC1    ICES1   -       WGM13   WGM12   CS12    CS11   CS10
TCCRxB  FOCxA    FOCxB   -       -       WGMx2   CSx2    CSx1    CSx0      Timer/Counter Control Register x B

TIMSKx  -        -       -       -       -       OCIExB  OCIExA  TOIEx     Timer/Counter Interrupt Mask Register
TIFRx   -        -       -       -       -       OCFxB   OCFxA   TOVx      Timer/Counter Interrupt Flag Register
TCNTx                                                                      Timer/Counter Register (stores the counter value)
OCRxA                                                                      Output Compare Register x A
OCRxB                                                                      Output Compare Register x B


*/
