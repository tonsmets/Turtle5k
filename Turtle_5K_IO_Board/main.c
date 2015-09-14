/*
 *	Module		: main.c
 *  System		: Turtle 5K I/O Board
 *  Version		: V01.00
 *	Date		: 01-07-2013
 *  Description	: Main file for controlling the I/O board
 *  Author		: Dirk-Jan Vethaak
 *
 *	History		:
 *
 */


/*
* INCLUDES
*/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <stdbool.h>
#include <util/delay.h>
#include "global.h"
#include "Shoot_Puls.h"
#include "Shoot_Level.h"
#include "Debug_UART.h"
#include "UART.h"
#include "Digital_Output.h"
#include "Digital_Input.h"
#include "ADC.h"

/*
* CONSTANTS
*/
#define WDT_PERIOD (0x07<<2)// define watchdog time 1 sec
#define WDT_ON false

/*
* LOCAL FUNCTION PROTOTYPES
*/
static void Clock_init();
static void Main_Init();
static void handle_received_packet( void );
static void set_adc_value_for_channel( uint8_t channel );
static void check_emergency_stop( void );

/*
* PUBLIC FUNCTIONS
*/

/*
 *	Function	: Main
 *  Author		: Dirk-Jan Vethaak
 *  Parameters	: none
 *	Return		: none
 *  Description	: The main function will first call all init functions. Then it will stay n the main loop.
 *				  when it received a packet then the main function will perform the action for that command. After
 *				  that it will send  a transmit packet to the host.
 *
 *	History		:
 *
 */
int main( void )
{
    Main_Init();

    while( 1 )
    {
#if WDT_ON
        // Kick the watchdog
        wdt_reset(); // reset wdt
#endif

        // Check if emergency signal is active
        check_emergency_stop();

        // Check the ADC channels (polling, without wait in function)
        adc_read_channels();

        // Handle debug interface commands
        debug_comm_data_received();



        if( UART_Packet_Received() )
        {

            handle_received_packet();

        } // end if has received

    } // end while

}	// end main

/*
* PRIVATE FUNCTIONS
*/

/*
 *	Function	: Initialization for the Main function
 *  Author		: Dirk-Jan Vethaak
 *  Parameters	: none
 *	Return		: none
 *  Description	: All the initialization function calls and global interrupt enable are
 *				  put together in this function
 *
 *	History		:
 *
 */
static void Main_Init()
{
    PORTE.OUTCLR = SHOOT_ENABLE_PM;
    PORTE.OUTCLR = SHOOT_TRIGGER_PM;

    Clock_init();		// Init the system clock to 16MHz
    initUSARTD0();		// Init Debug UART
    UART_init();		// Init the UART UART
    ADC_init();			// Init the ADC of port B

    Output_Init();		// Init the Digital Output
    Input_Init();		// Init the Digital Input and address Input

    Shoot_Puls_Init();	// Init the Shoot timer for the trigger pulse


    UART_Address_set( Input_Address_Get() ); // Set address from dip switches

#if WDT_ON
    wdt_enable( WDT_PERIOD );
#endif

    PMIC.CTRL = 0x07;  // interrupt levels high
    sei();			   //general interrupts enabled

    Shoot_Level_Init();	// Init the Shoot Level for the shoot steppe motor

    return;
}

/*
 *	Function	: Clock init
 *  Author		: Dirk-Jan Vethaak
 *  Parameters	: none
 *	Return		: none
 *  Description	: The clock will be configured to a external crystal of 16 MHz.
 *
 *	History		:
 *
 */
static void Clock_init()
{
    OSC.XOSCCTRL = 0xCB;	// select extern crystal
    OSC.CTRL |= OSC_XOSCEN_bm;

    while( !( OSC.STATUS & OSC_XOSCRDY_bm ) ); //wait until stable

    CCP = 0xD8 ;		// allow to write protected register
    CLK.CTRL = 0x03; // select extern crystal

    CCP = 0xD8 ;		// allow to write protected register
    CLK.PSCTRL = 0;

    return;
}

static void handle_received_packet( void )
{
    uint8_t bCommand = UART_get_packet_command(); // lower nibble

    // Set data applicable for all the send messages
    UART_set_tx_input( Input_get() );
    Output_Set( UART_get_output() ); //set output data

    // Select action from bCommand
    switch( bCommand )
    {
        case  CMD_SET_IO:
        {
            Shoot_Level_Set( UART_get_shoot_level() ); // set shoot level

            UART_set_tx_command( CMD_GET_IO );
            break;
        }

        case  CMD_SHOOT:
        {
            Shoot_Trigger( UART_shoot_trigger() ); // shoot trigger

            UART_set_tx_command( CMD_GET_IO );

            break;
        }

        case  CMD_READ_ADC_BALL_RIGHT_SINE:
        {
            Shoot_Level_Set( UART_get_shoot_level() ); // set shoot level

            UART_set_tx_command( CMD_GET_ADC_BALL_RIGHT_SINE );

            set_adc_value_for_channel( CH_BALL_RIGHT_SINE );

            break;
        }

        case  CMD_READ_ADC_BALL_LEFT_SINE:
        {
            Shoot_Level_Set( UART_get_shoot_level() ); // set shoot level

            UART_set_tx_command( CMD_GET_ADC_BALL_LEFT_SINE );

            set_adc_value_for_channel( CH_BALL_LEFT_SINE );

            break;
        }

        case  CMD_READ_ADC_BATTERY:
        {
            Shoot_Level_Set( UART_get_shoot_level() ); // set shoot level

            UART_set_tx_command( CMD_GET_ADC_BATTERY );

            set_adc_value_for_channel( CH_BATTERY );

            break;
        }

        case  CMD_READ_ADC_SHOOT:
        {
            Shoot_Level_Set( UART_get_shoot_level() ); // set shoot level

            UART_set_tx_command( CMD_GET_ADC_SHOOT );

            set_adc_value_for_channel( CH_SHOOT );

            break;
        }

        case CMD_HOME_POSITION:
        {
            Shoot_Level_go_home();
            UART_set_tx_command( CMD_GET_IO );

            break;
        }

#if false // Compass not implemented

        case  CMD_READ_COMPASS:
        {
            Shoot_Level_Set( UART_get_shoot_level() ); // set shoot level

            UART_set_tx_command( CMD_GET_MAG_X );


            // COMPASS_GET()
            // NOT IMPLEMENT IN THIS VERSION

            break;
        }

#endif

        default:
        {
            break;
        }
    }

    UART_Clr_Received();
    UART_Send_Packet();

    return;
}

static void set_adc_value_for_channel( uint8_t channel )
{
    uint8_t value[2];
    // Get the ADV value for the chosen channel
    ADC_GET( channel, value );

    UART_set_tx_adc( value[0], value[1] );

    return;
}


static void check_emergency_stop( void )
{
    if( Input_emergency_active() )
    {
        // Disable all outputs
        Output_all_off();

        // Stop shoot level
        Shoot_Level_emergency_stop();

        // Stop shoot pulse
        Shoot_Pulse_emergency_stop();
    }

    return;
}