
/*
 *	Module		: Shoot_Level.c
 *  System		: Turtle 5K I/O Board
 *  Version		: V01.00
 *	Date		: 02-07-2013
 *  Description	: Controls the steppenmotor driver for the Shoot Level.
 *				  Using an PWM signal for a specific time.
 *  Author		: Dirk-Jan Vethaak
 *
 *	History		:
 *
 */


// Include files
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include "global.h"
#include <stdbool.h>

#include "Digital_Input.h"

#include "Shoot_Level.h"

#define HOME_POSITION_INPUT 0x01

// Static variables
static uint8_t bPosition_Old;
static bool fPWM_Busy_Flag;
static int iSteps;



// PWM Clock is 16MHz, prescaler = 256
// Puls high = 64 us .   64us / (16MHz / 256 ) = 4 with microstep 2
#define PWM_DUTY_CYCLE 64;

// Puls lenght is for 14 full pulses. 14 is the scale factor for the number of steps indicated by one byte.
// +1 and -1 are to get it fixed halfway
// of the puls so that it not stops on the edge
//  14 *(PWM_duty_cycle +1)*2 -1  = 139
#define PULS_LENGHT 1819

// The shoot level is 0 at the lowest point ! Set the level
// before installing on the lowest position.

// Local function prototypes
static bool is_home( void );


/*
 *	Function	: Shoot_Level_Init
 *  Author		: Dirk-Jan Vethaak
 *  Parameters	: none
 *	Return		: none
 *  Description	: Initalisation of the peripherals of the
 *				  PWM (TCE1) and timer for the length of the PWM signal (TCF0).
 *
 *	History		:
 *
 */
void Shoot_Level_Init()
{

    // --- PWM SETUP -------------------------------------

    TCE1.CTRLB = 0x11;				// pwm mode
    TCE1.CCA = PWM_DUTY_CYCLE;		// Compare reg is PWM_Duty_Cycle
    TCE1.CTRLD = 0;					// diable events

    // --- TIMER SETUP for length PWM---------------------

    TCF0.CTRLB = 0 ;				// normal mode
    TCF0.CNT = 0 ;					// Forced counter to count up from 0
    TCF0.INTFLAGS = TC1_OVFIF_bm;
    TCF0.INTCTRLA = 0x07;			// enable interrupts high level

    TCF0.CTRLA = 0x00;				// disable PWM

    fPWM_Busy_Flag = false;			// Reset PWM busy flag

}

/* Emergency stop handling */
void Shoot_Level_emergency_stop( void )
{
    TCE1.CTRLA = 0x00;				// disable PWM
    TCF0.CTRLA = 0x00;				// disable PWM
    fPWM_Busy_Flag = false;			// Reset PWM busy flag

    return;
}

/* Make the stepper motor go to the home position */
void Shoot_Level_go_home( void )
{

    if( !Input_emergency_active() )
    {

        // Set the direction to go to home direction
        PORTE.OUTCLR = DIRECTION_PM;

        while( !is_home() )
        {
            // Keep stepping until home position is reached
            TCF0.PER = ( uint16_t )( PULS_LENGHT );	// puls lenght for 14 pulsen
            TCE1.CTRLA = 0x06;						// pwm aan CLK /256
            TCF0.CTRLA = 0x06;						// enable timer
        }

        // Set the direction to go away from the home direction as we already are home!
        PORTE.OUTSET = DIRECTION_PM;

        // Reset steps to ZERO
        iSteps = 0;

    }

    return;
}


/*
 *	Function	: Shoot_Level_Set
 *  Author		: Dirk-Jan Vethaak
 *  Parameters	: uint8_t bPosition, This is the value of 0-255 for the level height indication
 *	Return		: none
 *  Description	: The bPosition (0-255) is the height for the level height. The difference of the new and old value will
 *				  be the number of steps. This byte will be multiplied
 *				  by a scale factor constant to scale it from a byte to maximum 3565 pulses. The direction output pin is set
 *				  high if difference is positive or low when it is negative difference
 *				  The PWM and timer peripheral are started. The PWM will stop when the timer (TCF0) overflows.
 *
 *	History		:
 *
 */
void Shoot_Level_Set( uint8_t bPosition )
{
    // Only set Level when PWM is not busy yet and emergency signal is not active

    if( !fPWM_Busy_Flag && !Input_emergency_active() )
    {
        iSteps = bPosition - bPosition_Old ;		// Calc signed difference
        bPosition_Old = bPosition;					// Save old position

        if( iSteps > 0 )								// positive difference
        {
            PORTE.OUTSET = DIRECTION_PM;
        }

        else										// negative difference
        {
            PORTE.OUTCLR = DIRECTION_PM;
            iSteps = abs( iSteps ) ;					// |iTemp|
        }

        if( iSteps > 0 )								// Only start timers when iSteps > 0
        {

            TCF0.PER = ( uint16_t )( PULS_LENGHT );	// puls lenght for 14 pulsen
            TCE1.CTRLA = 0x06;						// pwm aan CLK /256
            TCF0.CTRLA = 0x06;						// enable timer

            fPWM_Busy_Flag = true ;

        }

    }
}

void Shoot_Level_do_step( void )
{

    TCF0.PER = ( uint16_t )( PULS_LENGHT );	// puls lenght for 14 pulsen
    TCE1.CTRLA = 0x06;						// pwm aan CLK /256
    TCF0.CTRLA = 0x06;						// enable timer

    return;
}

/*
 *	Function	: ISR TIMER F0 Overflow
 *  Author		: Dirk-Jan Vethaak
 *  Parameters	: TCF0_OVF_vect, interrupt vector from the timer TCF0 overflow.
 *	Return		: none
 *  Description	: When the timer TCF0 overflows the PWM peripheral will be stopt. The timer peripheral will also be reseted
 *				  The next interrupt is only possible after a full generated pwm signal.
 *
 *	History		:
 *
 */

ISR( TCF0_OVF_vect )
{


    iSteps--;

    TCE1.CTRLA = 0x00;		// disable PWM

    TCF0.CTRLA =  0x00 ;	// disable interrupt.
    TCF0.CNT = 0 ;			// reset count

    if( iSteps > 0 ) {

        // next pulsstream of 14 pulses
        TCF0.PER = ( uint16_t )( PULS_LENGHT );		// puls lenght for 14 pulsen
        TCE1.CTRLA = 0x06;						   // pwm aan CLK /256
        TCF0.CTRLA = 0x06;

    }

    else
    {
        fPWM_Busy_Flag = false;
        iSteps = 0;

    }

    PORTE.OUTCLR = PWM_PM; // Always reset PWM pin output for avoiding high state.

}

static bool is_home( void )
{
    return ( Input_get() & HOME_POSITION_INPUT );

}
