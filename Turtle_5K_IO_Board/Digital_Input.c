/*
 *	Module		: Digital_Input.c
 *  System		: Turtle 5K I/O Board
 *  Version		: V01.00
 *	Date		: 02-07-2013
 *  Description	: Controls and read the digital Input data pins.
 *  Author		: Dirk-Jan Vethaak
 *
 *	History		:
 *
 */

#include <avr/io.h>
#include <stdbool.h>
#include "global.h"
#include "Digital_Input.h"

#define PORTA_MASK	0xE0
#define PORTB_MASK	0xE0
#define PORTD_MASK  0xF3
#define PORTE_MASK  0x04

/*
 *	Function	: Input_Init
 *  Author		: Dirk-Jan Vethaak
 *  Parameters	: none
 *	Return		: none
 *  Description	: Initalisation of digital input pins and address pin connected to the dipswitches.
 *				  PA5/6/7 and PB5/6/7 and PD0/1/4/5/6/7 and PE2 (emergency input)
 *
 *	History		:
 *
 */
void Input_Init()
{

    PORTA.OUTCLR = PORTA_MASK;  // Clear data pins, no pull up.
    PORTA.DIRCLR = PORTA_MASK;  // Set data pins as input

    PORTB.OUTCLR = PORTB_MASK;  // Clear data pins, no pull up.
    PORTB.DIRCLR = PORTB_MASK;  // Set data pins as input

    PORTD.OUTCLR = PORTD_MASK;  // Clear data pins, no pull up.
    PORTD.DIRCLR = PORTD_MASK;  // Set data pins as input

    PORTE.OUTCLR = PORTE_MASK;  // Clear data pins, no pull up.
    PORTE.DIRCLR = PORTE_MASK;  // Set data pins as input

    return;
}


/*
 *	Function	: Input_Get
 *  Author		: Dirk-Jan Vethaak
 *  Parameters	: none
 *	Return		: Input data on PortA
 *  Description	: Returns the data on portA. The data is pulled up at the board. So the data will be
 *				  inverted to get an '1' for active , '0' for not active.
 *
 *	History		:
 *
 */
uint8_t Input_get()
{
    uint8_t bByte_In = ( PORTA.IN & PORTA_MASK ) >> 5;

    bByte_In |= ( ( PORTB.IN & PORTB_MASK ) >> 2 );
    bByte_In |= ( ( PORTD.IN & PORTD_MASK ) << 6 );

    bByte_In ^= 0xFF;	// Toggle bits

    return bByte_In;
}


/*
 *	Function	: Input_Address_Get
 *  Author		: Dirk-Jan Vethaak
 *  Parameters	: none
 *	Return		: uint8_t address,high nibble ( low nibble will be command)
 *  Description	: Gets the data op the dipswitches for the input used for the address.
 *
 *	History		:
 *
 */
uint8_t Input_Address_Get()
{
    uint8_t bAddress = ( PORTD.IN & 0xF0 );	// Upper nibble of PortD are the Address bits
    bAddress = bAddress ^ 0xF0;		// Toggle all address bits, used to get the same data as dipswitch (on = 1)

    return bAddress;		// returns address byte


}

bool Input_emergency_active( void )
{
    if( PORTE.IN & 0x04 )
    {
        return true;
    }

    return false;
}