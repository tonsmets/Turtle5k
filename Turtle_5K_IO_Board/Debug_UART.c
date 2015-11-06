/*
 *	Module		: Debug_UART.c
 *  System		: Turtle 5K I/O Board
 *  Version		: V0.2
 *	Date		: 02-07-2013
 *  Description	: Controls the Debug UART
 *
 *  Author		: Dirk-Jan Vethaak
 *
 *	History		:
 *
 */

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdlib.h>
#include "global.h"
#include "ADC.h"
#include "Digital_Input.h"
#include "Digital_Output.h"
#include "Shoot_Level.h"
#include "Shoot_Puls.h"
#include "usart_driver.h"
#include "Debug_UART.h"

#define enter 	0x0D
#define BS		0x08	// backspace character
#define space	0x20	// space character

#define C_CMD_VERSION       		'['		// software version
#define C_CMD_ADC_CH0       		'0'		// ADC CH0
#define C_CMD_ADC_CH1				'1'		// ADC CH1
#define C_CMD_ADC_CH2				'2'		// ADC CH2
#define C_CMD_ADC_CH3	   			'3'     // ADC CH3
#define C_CMD_ACTIVATE_OUTPUTS		'I'		// All outputs on
#define C_CMD_DEACTIVATE_OUTPUTS	'O'		// All outputs off
#define C_CMD_EMERGENCY_STATE		'E'		// Check status of emergency signal
#define C_CMD_READ_INPUTS			'R'		// Read input status
#define C_CMD_GO_HOME				'H'		// Read input status
#define C_CMD_DO_STEP				'S'		// Do one step
#define C_CMD_CCW					'F'		// Rotate CCW
#define C_CMD_CW					'B'		// Rotate CW
#define C_CMD_SHOOT					'K'		// Do one shoot
#define C_CMD_TOGGLE_SHOOT_ENABLE	'T'		// Do one shoot

#define USART USARTD0

static USART_data_t USART_data;
static uint8_t sReply[64];


static void debug_comm_handler_cmd( uint8_t c_in );

/*
 *	Function	: init UART
 *  Author		: Dirk-Jan Vethaak
 *  Parameters	: none
 *	Return		: none
 *  Description	: Initialisation of UART for full duplex, baudrate of 115200.
 *
 *	History		:
 *
 */
void initUSARTD0( void )
{
    //PORTD_OUTSET		= 0x08;
    PORTD.DIRSET		= PIN3_bm; 	// configure TX/D3 for output, RX/D2 for input
    PORTD.DIRCLR		= PIN2_bm;

    USART_InterruptDriver_Initialize( &USART_data, &USART, USART_DREINTLVL_LO_gc );

    /* USARTC0, 8 Data bits, No Parity, 1 Stop bit. */
    USART_Format_Set( USART_data.usart, USART_CHSIZE_8BIT_gc,
                      USART_PMODE_DISABLED_gc, false );

    /* Enable RXC interrupt. */
    USART_RxdInterruptLevel_Set( USART_data.usart, USART_RXCINTLVL_LO_gc );

    /* Set Baudrate to 115k2 bps:
        * Use the default I/O clock frequency that is 16 MHz.
        * Do not use the baudrate scale factor
        *
        * Baudrate select = (1/(16*(((I/O clock frequency)/Baudrate)-1)
        *                 = 983
        */
    USART_Baudrate_Set( &USART, 983 , -7 ); // 115k2

    /* Enable both RX and TX. */
    USART_Rx_Enable( USART_data.usart );
    USART_Tx_Enable( USART_data.usart );

    debug_comm_handler_cmd( C_CMD_VERSION );

}

ISR( USARTD0_RXC_vect )
{
    USART_RXComplete( &USART_data );
}

ISR( USARTD0_DRE_vect )
{
    USART_DataRegEmpty( &USART_data );
}

// ------------------------------------------------------------------------------------------------------
// The following subroutine outputs a character less than 9 bits to the
// USARTD0 module using the polling method.
// ------------------------------------------------------------------------------------------------------
/*
 *	Function	: putchUSARTD0
 *  Author		: Dirk-Jan Vethaak
 *  Parameters	: char x , Byte to send
 *	Return		: none
 *  Description	: transmit a byte by the UART
 *
 *	History		:
 *
 */
void putchUSARTD0( char cx )
{
    while( !( USARTD0.STATUS & USART_DREIF_bm ) );

    USARTD0.DATA = cx;

}

// ------------------------------------------------------------------------------------------------------------------------
// This function outputs a string to the USARTE0 by calling the putchUSARTE0 continuously
// until the whole string has been sent out.
// ------------------------------------------------------------------------------------------------------------------------
/*
 *	Function	: putchUSARTD0
 *  Author		: Dirk-Jan Vethaak
 *  Parameters	: ptr char[]  , Bytes to send
 *	Return		: none
 *  Description	: transmit a  byte array by UART
 *
 *	History		:
 *
 */
void putsUSARTD0( uint8_t *ptr )
{

    while( *ptr )
    {
        bool byteTobuffer = USART_TXBuffer_PutByte( &USART_data, *ptr );

        if( byteTobuffer )
        {
            ptr++;
        }
    }

    while( !( USART_TXBuffer_PutByte( &USART_data, '\n' ) ) );

    return;
}

void debug_comm_data_received( void )
{
    if( USART_RXBufferData_Available( &USART_data ) )
    {
        debug_comm_handler_cmd( USART_RXBuffer_GetByte( &USART_data ) );
    }

    return;
}
/**************************************************************************************************
 @fn      debug_comm_handler_cmd

 @brief   Handles the debug commands

 @return  None
**************************************************************************************************/
static void debug_comm_handler_cmd( uint8_t c_in )
{

    switch( c_in )
    {
        case C_CMD_VERSION:
        {
            sReply [0]  = 'I';
            sReply [1]  = 'O';
            sReply [2]  = ' ';
            sReply [3]  = 'B';
            sReply [4]  = 'O';
            sReply [5]  = 'A';
            sReply [6]  = 'R';
            sReply [7]  = 'D';
            sReply [8]  = ' ';
            sReply [9]  = 'V';
            sReply [10]  = C_VERSION_MAIOR_1;
            sReply [11]  = C_VERSION_MAIOR_2;
            sReply [12]  = '.';
            sReply [13]  = C_VERSION_MINOR;
            sReply [14] = C_VERSION_MIN;
            sReply [15] = '\n';
            sReply [16] = '\0';
            break;
        }

        case C_CMD_ADC_CH0:
        {
            uint8_t value[2];
            ADC_GET( CH_BALL_RIGHT_SINE, value );

            int16_t res = value[0] << 8 | value[1];

            itoa( ( int )res, ( char* ) &sReply[4], 10 );

            sReply[0] = 'C';
            sReply[1] = 'H';
            sReply[2] = '0';
            sReply[3] = ' ';

            break;
        }

        case C_CMD_ADC_CH1:
        {
            uint8_t value[2];
            ADC_GET( CH_BALL_LEFT_SINE, value );

            int16_t res = value[0] << 8 | value[1];

            itoa( res, ( char* )&sReply[4], 10 );

            sReply[0] = 'C';
            sReply[1] = 'H';
            sReply[2] = '1';
            sReply[3] = ' ';

            break;
        }

        case C_CMD_ADC_CH2:
        {
            uint8_t value[2];

            ADC_GET( CH_BATTERY, value );

            int16_t res = value[0] << 8 | value[1];

            itoa( res, ( char* ) &sReply[4], 10 );

            sReply[0] = 'C';
            sReply[1] = 'H';
            sReply[2] = '2';
            sReply[3] = ' ';

            break;
        }

        case C_CMD_ADC_CH3:
        {
            uint8_t value[2];
            ADC_GET( CH_SHOOT, value );

            int16_t res = value[0] << 8 | value[1];

            itoa( res, ( char* )&sReply[4], 10 );

            sReply[0] = 'C';
            sReply[1] = 'H';
            sReply[2] = '3';
            sReply[3] = ' ';

            break;
        }

        case C_CMD_ACTIVATE_OUTPUTS:
        {
            PORTE.OUTSET = 0xFB;
            PORTF.OUTSET = 0xFF;
            sReply[0] = 'A';
            sReply[1] = 'L';
            sReply[2] = 'L';
            sReply[3] = ' ';
            sReply[4] = 'O';
            sReply[5] = 'N';
            sReply[6] = '\0';

            break;
        }

        case C_CMD_DEACTIVATE_OUTPUTS:
        {
            PORTE.OUTCLR = 0xFB;
            PORTF.OUTCLR = 0xFF;
            sReply[0] = 'A';
            sReply[1] = 'L';
            sReply[2] = 'L';
            sReply[3] = ' ';
            sReply[4] = 'O';
            sReply[5] = 'F';
            sReply[6] = 'F';
            sReply[7] = '\0';

            break;
        }

        case C_CMD_EMERGENCY_STATE:
        {

            if( Input_emergency_active() )
            {
                sReply[0] = 'E';
                sReply[1] = 'M';
                sReply[2] = ' ';
                sReply[3] = 'O';
                sReply[4] = 'N';
                sReply[5] = '\0';

            }

            else
            {
                sReply[0] = 'E';
                sReply[1] = 'M';
                sReply[2] = ' ';
                sReply[3] = 'O';
                sReply[4] = 'F';
                sReply[5] = 'F';
                sReply[6] = '\0';
            }

            break;
        }

        case C_CMD_READ_INPUTS:
        {
            uint8_t inputs = Input_get();

            sReply[0] = 'I';
            sReply[1] = 'N';
            sReply[2] = 'P';
            sReply[3] = 'U';
            sReply[4] = 'T';
            sReply[5] = ' ';
            sReply[6] = inputs;
            sReply[7] = '\0';

            break;
        }

        case C_CMD_GO_HOME:
        {
            Shoot_Level_go_home();

            sReply[0] = 'G';
            sReply[1] = 'O';
            sReply[2] = ' ';
            sReply[3] = 'H';
            sReply[4] = 'O';
            sReply[5] = 'M';
            sReply[6] = 'E';
            sReply[7] = '\0';

            break;

        }

        case C_CMD_DO_STEP:
        {
            Shoot_Level_do_step();

            sReply[0] = 'D';
            sReply[1] = 'O';
            sReply[2] = ' ';
            sReply[3] = 'S';
            sReply[4] = 'T';
            sReply[5] = 'E';
            sReply[6] = 'P';
            sReply[7] = '\0';

            break;

        }

        case C_CMD_CCW:
        {
            PORTE.OUTSET = DIRECTION_PM;

            sReply[0] = 'S';
            sReply[1] = 'T';
            sReply[2] = 'E';
            sReply[3] = 'P';
            sReply[4] = ' ';
            sReply[5] = 'C';
            sReply[6] = 'C';
            sReply[7] = 'W';
            sReply[8] = '\0';

            break;

        }

        case C_CMD_CW:
        {
            PORTE.OUTCLR = DIRECTION_PM;

            sReply[0] = 'S';
            sReply[1] = 'T';
            sReply[2] = 'E';
            sReply[3] = 'P';
            sReply[4] = ' ';
            sReply[5] = 'C';
            sReply[6] = 'W';
            sReply[7] = '\0';

            break;

        }

        case C_CMD_SHOOT:
        {
            // Wait for shoot to finish
            while( Shoot_is_charging() );

            // Set shoot pulse
            Shoot_Trigger( 0x7A );

            sReply[0] = 'F';
            sReply[1] = 'I';
            sReply[2] = 'R';
            sReply[3] = 'E';
            sReply[4] = '\0';

            break;
        }

        case C_CMD_TOGGLE_SHOOT_ENABLE:
        {
            // Toggle J7_3 PE5
            PORTE_OUTTGL = SHOOT_ENABLE_PM;

            if( PORTE_IN & SHOOT_ENABLE_PM )
            {
                sReply[0] = 'O';
                sReply[1] = 'N';
                sReply[2] = ' ';
            }

            else
            {

                sReply[0] = 'O';
                sReply[1] = 'F';
                sReply[2] = 'F';

            }

            sReply[3] = '\0';
            break;
        }

        default:
        {
            sReply[0] = '?';
            sReply[1] = '?';
            sReply[2] = '\0';

            break;
        }
    }


    putsUSARTD0( sReply );

    return;
}