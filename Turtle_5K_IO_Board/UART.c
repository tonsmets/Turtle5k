/*
 *	Module		: UART.c
 *  System		: Turtle 5K I/O Board
 *  Version		: V01.00
 *	Date		: 01-07-2013
 *  Description	: Controls the UART for the RS422 interface. It is using
 *				  interrupts for a fast timing.
 *  Author		: Dirk-Jan Vethaak
 *
 *	History		:
 *
 */

// Include files
#include <avr/interrupt.h>
#include <avr/io.h>
#include "global.h"
#include "UART.h"

#define Packet_lenght 6
#define SOF 'Z'
#define EOF '\r'

// Global variables
// Receive buffer
uint8_t volatile abBuffer_rx[Packet_lenght];
// transmit buffer
uint8_t volatile abBuffer_tx[Packet_lenght];

// Static variables
static bool fPacket_Recieved_flag = false;
static bool fLoad_RX_flag = false;
static bool fSof_Received_flag = false;
static uint8_t bAddress = 0;
static uint8_t bCounter_TX = 0;
static uint8_t bCounter_RX = 0;


/*
 *	Function	: RS422-init
 *  Author		: Dirk-Jan Vethaak
 *  Parameters	: none
 *	Return		: none
 *  Description	: Initialisation of UART for full duplex, baudrate of 115200.
 *
 *	History		:
 *
 */
void UART_init()
{

    PORTC.DIRSET 	= 0x08; // configure TX/PC3 for output, RX/PC2 for input

    // set baud rate to 115200 with fPER = 16 MHz. Bsel = 554 and bscale = -6 0b1001
    USARTC0.BAUDCTRLA = 0xD7;	// Bsel  low byte
    USARTC0.BAUDCTRLB = 0x93;	// Bscale nibble & Bsel high nibble

    USARTC0.CTRLA	= 0x30;	// enable USARTE0 interrupts RX, high priority
    USARTC0.CTRLB	= 0x18;	// enable both transmitter and receiver
    USARTC0.CTRLC	= 0x03;	// select asynchronous USART, disable parity, 1 stop bit, 8 data bits

    // TX enable of UART chip driver
    PORTC.DIRSET = RS422_TX_ENABLE_PM;		// PC4
    PORTC.OUTSET = RS422_TX_ENABLE_PM;		// TX enable of RS422 driver is set



    return;
}


/*
 *	Function	: UART_putch
 *  Author		: Dirk-Jan Vethaak
 *  Parameters	: char , one character or byte to send
 *	Return		: none
 *  Description	: Transmits one byte
 *
 *	History		:
 *
 */
void UART_putch( char cx )
{
    //PORTE_OUTSET = 0x02;
    while( !( USARTC0.STATUS & USART_DREIF_bm ) ); // if hangs watchdog will reset.

    USARTC0.DATA = cx;

}

/*
 *	Function	: UART_puts
 *  Author		: Dirk-Jan Vethaak
 *  Parameters	: char * ptr , start pointer of String
 *	Return		: none
 *  Description	: Transmits a String. No interrupt
 *
 *	History		:
 *
 */
void UART_puts( char *ptr )
{
    while( *ptr ) {
        UART_putch( *ptr );
        ptr++;
    }
}

/*
 *	Function	: UART_getch
 *  Author		: Dirk-Jan Vethaak
 *  Parameters	: none
 *	Return		: char
 *  Description	: will return the byte received by the UART.
 *				  Function will wait till one byte is received
 *
 *	History		:
 *
 */
char UART_getch( void )
{
    while( !( USARTC0.STATUS & USART_RXCIF_bm ) ); // wait for a new character, if hangs watchdog will reset.

    return USARTC0.DATA;	// fetch the received character
}

/*
 *	Function	: UART_has_received
 *  Author		: Dirk-Jan Vethaak
 *  Parameters	: none
 *	Return		: Flag byte if there is a valid packet received
 *  Description	: Returns the byte flag if there is a valid packet received.
 *
 *	History		:
 *
 */
bool UART_Packet_Received() {

    return fPacket_Recieved_flag;
}

/*
 *	Function	: UART_Clr_Received
 *  Author		: Dirk-Jan Vethaak
 *  Parameters	: none
 *	Return		: none
 *  Description	: Clear the received packet flag
 *
 *	History		:
 *
 */
void UART_Clr_Received() {

    fPacket_Recieved_flag = false;

    return;
}

/*
 *	Function	: UART_Send_Packet
 *  Author		: Dirk-Jan Vethaak
 *  Parameters	: none
 *	Return		: none
 *  Description	: Function starts transmitting the packet. After that the
 *				  first byte is send the interrupt routine TX will
 *				  transmit the other bytes.
 *
 *	History		:
 *
 */
void UART_Send_Packet()
{

    // Setup SOF and EOF
    abBuffer_tx[0] = SOF;
    abBuffer_tx[Packet_lenght - 1] = EOF;

    // enable interrrupt tx but first load new data!
    UART_putch( abBuffer_tx[0] );
    USARTC0.CTRLA	= 0x3C;  // enable RX interrupt high priority and TX interrupt high priority

    bCounter_TX = 0;

    return;
}

/*
 *	Function	: Shoot_Puls-init
 *  Author		: Dirk-Jan Vethaak
 *  Parameters	: Interrupt uart TX vector
 *	Return		: none
 *  Description	: Interrupt routine when a byte is transmitted by the UART.
 *				  While not all bytes are transmitted it will transmit the next bytes in the buffer array
 *				  Before transmitting the last byte, the interrrupt will be disabled.
 *				  No more bytes will be transmitted
 *	History		:
 *
 */
ISR( USARTC0_TXC_vect )
{

    bCounter_TX ++;

    if( bCounter_TX < Packet_lenght - 1 )
    {
        UART_putch( abBuffer_tx[bCounter_TX] ); // load next

    }

    if( bCounter_TX >= ( Packet_lenght - 1 ) )
    {

        // disable  interrrupt tx
        USARTC0.CTRLA	= 0x30; // enable RX interrupt high priority
        UART_putch( abBuffer_tx[Packet_lenght - 1] ); // send EOF

    }

    return;
}
/*
 *	Function	: Interrupt Routine for Receive buffer is full.
 *  Author		: Dirk-Jan Vethaak
 *  Parameters	: interrrupt UART receive buffer is full vector
 *	Return		: none
 *  Description	: Interrrupt routine when a byte is received by the UART.
 *				  The byte will be compared to the SOF and bAddress if ok fill
 *				  rest of buffer, otherwise compare till SOF and bAddress are ok.
 *
 *	History		:
 *
 */
ISR( USARTC0_RXC_vect ) {


    uint8_t bTemp = USARTC0.DATA; // read buffer

    // if start of header received check address
    if( fSof_Received_flag ) {



        if( ( bTemp & 0xF0 ) == bAddress ) // upper nibble is address, low nibble is command
        {

            fLoad_RX_flag = true; // set load flag after received Sof and Address
            bCounter_RX ++;

        } else {
            // no correct address.
            bCounter_RX = 0;
            fLoad_RX_flag = false;

        }
    }

    // Compare Byte if SOF
    if( ( bTemp == SOF ) )
    {



        if( !fLoad_RX_flag )
        {
            // start header detected and not in packet self
            bCounter_RX = 0;
            fSof_Received_flag = true; // SOF received

        } else {
            // SOF received while loading.. perhaps wrong start point.
            // NOT IMPLEMETED
            // Read next SOF and address combination

        }

    }

    // Load rest of buffer after received SOF and address
    if( fLoad_RX_flag )
    {

        fSof_Received_flag = false;  // reset flag

        abBuffer_rx[bCounter_RX] = bTemp;  // load data in buffer
        bCounter_RX++;


        // check if EOf is received while loading buffer

        if( abBuffer_rx[Packet_lenght - 1] == EOF )
        {
            bCounter_RX = 0;		// reset counter
            fLoad_RX_flag = false;
            fPacket_Recieved_flag = true;

            abBuffer_rx[Packet_lenght - 1] = '0'; // make sure not stay in buffer


        }

        // If no EOF found -> clear buffer , wait till next packet
        if( bCounter_RX > ( Packet_lenght ) )
        {

            bCounter_RX = 0;
            fLoad_RX_flag = false;
            fSof_Received_flag = false;
        }

    }

    return;
}

/*
 *	Function	: UART_get_packet_command
 *  Author		: Dennis Wokke
 *  Parameters	: none
 *	Return		: lower nibble of the second byte in the packet
 *  Description	: Get the command form the received packet
 *
 *	History		:
 *
 */
uint8_t UART_get_packet_command( void )
{
    return abBuffer_rx[1] & 0x0F; // lower nibble
}

/*
 *	Function	: UART_get_output
 *  Author		: Dennis Wokke
 *  Parameters	: none
 *	Return		: Third byte in the packet
 *  Description	: Get the value to set to the outputs from the packet
 *
 *	History		:
 *
 */
uint8_t UART_get_output( void )
{
    return abBuffer_rx[2];
}

/*
 *	Function	: UART_get_output
 *  Author		: Dennis Wokke
 *  Parameters	: none
 *	Return		: Fourth byte in the packet
 *  Description	: Get the value to set the shoot trigger from the packet
 *
 *	History		:
 *
 */
uint8_t UART_shoot_trigger( void )
{
    return abBuffer_rx[3];
}

/*
 *	Function	: UART_get_shoot_level
 *  Author		: Dennis Wokke
 *  Parameters	: none
 *	Return		: Fifth byte in the packet
 *  Description	: Get the value for the shoot level from the packet
 *
 *	History		:
 *
 */
uint8_t UART_get_shoot_level( void )
{
    return abBuffer_rx[4];
}

void UART_set_tx_command( uint8_t cmd )
{
    abBuffer_tx[1] =  bAddress | cmd;

    return;
}

void UART_set_tx_input( uint8_t input )
{
    abBuffer_tx[2] = input;

    return;
}

void UART_set_tx_adc( uint8_t adc_h, uint8_t adc_l )
{
    abBuffer_tx[3] = adc_h;
    abBuffer_tx[4] = adc_l;

    return;
}

/*
 *	Function	: UART_Adress_set
 *  Author		: Dirk-Jan Vethaak
 *  Parameters	: byte adress ( high nibble)
 *	Return		: none
 *  Description	: sets the address to compare the RS422 datapacket
 *
 *	History		:
 *
 */
void UART_Address_set( uint8_t bAdr ) {

    bAddress = bAdr;

}