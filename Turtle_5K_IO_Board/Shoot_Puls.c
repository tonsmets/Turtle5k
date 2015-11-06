/*
 *	Module		: Shoot_Puls.c
 *  System		: Turtle 5K I/O Board
 *  Version		: V01.00
 *	Date		: 02-07-2013
 *  Description	: Controls the trigger puls for the Shoot module in the Turtle 5K for a specific time.
 *				  0- 30 ms
 *  Author		: Dirk-Jan Vethaak
 *
 *	History		:
 *
 */


#include <avr/interrupt.h>
#include <avr/io.h>
#include "global.h"
#include <stdbool.h>

// Static variables
static bool fShoot_Busy_Flag;


/*
 *	Function	: Shoot_Puls-init
 *  Author		: Dirk-Jan Vethaak
 *  Parameters	: none
 *	Return		: none
 *  Description	: Initialization of the trigger i/o pin and the timer TCE0 as normal
 *				  timer with interrupt.
 *
 *	History		:
 *
 */
void Shoot_Puls_Init()
{

    TCE0.CTRLB = 0;					// operate in normal mode
    TCE0.CNT = 0 ;					// Forced counter to count up from 0
    TCE0.INTFLAGS = TC1_OVFIF_bm;	// Clear Interrupts
    TCE0.INTCTRLA = 0x03;			// enable interrupts high level

    fShoot_Busy_Flag = false;				// Clear Flag
}

void Shoot_Pulse_emergency_stop( void )
{
    PORTE.OUTCLR = SHOOT_TRIGGER_PM; // reset pin
    TCE0.CTRLA =  0x00 ; // disable interrupt.
    fShoot_Busy_Flag = false;
    return;
}

/*
 *	Function	: Shoot_Trigger
 *  Author		: Dirk-Jan Vethaak
 *  Parameters	: uint8_t time, This is the value of 0-255 for the shoot puls indication
 *	Return		: none
 *  Description	: When the trigger is set with a specific
 *				  time the pin goes high for this period. When the Timer TEC0 overflows the interrupt
 *				  will stop the timer and reset the pin. There is only one shot at the time.
 *
 *	History		:
 *
 */

void Shoot_Trigger( uint8_t time )
{

    // only one shoot trigger puls at the time
    if( !fShoot_Busy_Flag )
    {

        fShoot_Busy_Flag = true;
        // set top value, prescaler is 1024 and CLK is 16MHz.
        // 0- 255 -> 0 - 32.5 ms. The time of 255 counts is 16.25 ms, so a
        // multiply by 2 * time is needed to get  it to 32.5ms
        TCE0.PER = time << 1 ;

        PORTE.OUTSET = SHOOT_TRIGGER_PM;
        TCE0.CTRLA =  0x07;		// start enable timer, prescaleer = 1024


    }
}

bool Shoot_is_charging( void )
{
    return fShoot_Busy_Flag;
}

/*
 *	Function	: Interrupt Routine for Overflow of timer used for the shoot puls length.
 *  Author		: Dirk-Jan Vethaak
 *  Parameters	: Overflow interrupt vector.
 *	Return		: none
 *  Description	: The overflow interrupt will be called if the timer overflows. Then the timer is
 *				  stopped en the interrupt is disabled. The trigger pin is cleared. At the end
 *				  the flag of shoot busy will be cleared, avoiding a trigger will the shoot puls is busy.
 *
 *	History		:
 *
 */
ISR( TCE0_OVF_vect )
{

    PORTE.OUTCLR = SHOOT_TRIGGER_PM; // reset pin
    TCE0.CTRLA =  0x00 ; // disable interrupt.
    TCE0.CNT = 0 ;		 // reset timer.
    fShoot_Busy_Flag = false;

}