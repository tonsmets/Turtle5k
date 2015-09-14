/*
 *	Module		: Digital_Output.c
 *  System		: Turtle 5K I/O Board
 *  Version		: V01.00
 *	Date		: 02-07-2013
 *  Description	: Controls the digital output data pins. Data is send by the host. The
 *				  Shoot_level and shoot_trigger are overruling the pins
 *  Author		: Dirk-Jan Vethaak
 *
 *	History		:
 *
 */

#include <avr/io.h>
#include "global.h"
#include "Debug_UART.h"
#include "Digital_Input.h"
#include "Digital_Output.h"

#define PORTE_MASK 0xE2		// Filter ports with fixed pins
#define PORTF_MASK 0x0F		// Filter ports with fixed pins


/*
 *	Function	: Output_Init
 *  Author		: Dirk-Jan Vethaak
 *  Parameters	: none
 *	Return		: none
 *  Description	: Initialisation of digital Output pins
 *
 *	History		:
 *
 */
void Output_Init()
{
    PORTE.OUTCLR = 0xFF;	// Clear all data output pins
    PORTE.DIRSET = 0xFB;	// Set PortE as output, except pin PE2 (emergency signal)

    PORTF.OUTCLR = 0xFF;	// Clear all data output pins
    PORTF.DIRSET = 0xFF;	// Set PortF as output

}

/*
 *	Function	: Output_Set
 *  Author		: Dirk-Jan Vethaak
 *  Parameters	: uint8_ bdata
 *	Return		: none
 *  Description	: Set the data out pins. The pwm, direction and the shoot puls are overruling this port.
 *				  PWM , direction and shoot trigger pin are defined in the c files.
 *
 *				  The data bits are shift on the ports because of the fixed pins for the PWM , Shoot
 *				  trigger and Direction for the steppenmotor.
 *
 *				  PE0:	Shoot Trigger*			|   PF0: Data_out[4]
 *				  PE1:  Data_out[0]				|   PF1: Data_out[5]
 *				  PE2:  Emergency input!!		|   PF2: Data_out[6]
 *				  PE3:  Shoot level PWM*		|   PF3: Data_out[7]
 *				  ----------------------------------------------------
 *	    		  PE4:  Shoot level direction*	|   PF4:
 *				  PE5:  Shoot enable Data_out[1]|   PF5:
 *				  PE6:  Data_out[2]				|   PF6:
 *				  PE7:  Data_out[3]				|   PF7:
 *
 *				  * fixed hardware pin
 *				  !! is an input
 *
 *				 uint8_t bDate bitmap:
 *				 Bit	|	Pin
 *				 ----------------------
 *				 0		|	PE1
 *				 1		|	PE5	(Shoot_enable)
 *				 2		|	PE6
 *				 3		|	PE7
 *				 4		|	PF0
 *				 5		|	PF1
 *				 6		|	PF2
 *				 7		|	PF3
 *
 *				 Because the 8 data output en the fixed pins are on the same port.
 *				 There are 3 nibble masks to set or reset the bits.
 *
 *
 *	History		:
 *
 */
void Output_Set( const uint8_t bData )
{
    if( !Input_emergency_active() )
    {   // Emergency signal not active, so outputs can be set

        uint8_t bNibble_0_Low = bData & 0x0F;					// bits 0-4 Port PE
        uint8_t bNibble_0_High = ( bData & 0xF0 ) >> 4;			// bits 4-7 Port PF

        uint8_t bSet_bm = 0;  // bitmask for set bits
        uint8_t bClr_bm = 0;  // bitmask for clear bits

        // Calculate which pins to set for port E
        bSet_bm = ( bNibble_0_Low & 0x01 ) << 1; // PE1

        bSet_bm |= ( bNibble_0_Low & 0x0E ) << 4; // PE5 - PE7

        // Set pins to set
        bSet_bm = ( bSet_bm ) & PORTE_MASK;			// filter fixed pins out

        // Set pins to clear
        bClr_bm = ( ~( bSet_bm ) ) & PORTE_MASK;	// filter fixed pins out

        // Set port E outputs
        PORTE.OUTSET = bSet_bm;
        PORTE.OUTCLR = bClr_bm;

        // Set port F outputs 0-3
        PORTF.OUT = ( bNibble_0_High & PORTF_MASK );

    }

    else
    {   // Emergency active, so all outputs to zero!
        PORTE.OUTCLR = 0xFB;
        PORTF.OUTCLR = 0xFF;
    }

    return;

}

void Output_all_off( void )
{
    PORTE.OUTCLR = 0xFB;
    PORTF.OUTCLR = 0xFF;

    return;
}