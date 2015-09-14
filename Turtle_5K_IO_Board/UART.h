/*
 *	Module		: UART.h
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

#ifndef UART_H_
#define UART_H_

void UART_init( void );
void UART_puts( char *ptr );
bool UART_Packet_Received();
void UART_Clr_Received();
char* UART_get_buffer();
void UART_Address_set( uint8_t bAdr );

uint8_t UART_get_packet_command( void );
uint8_t UART_get_output( void );
uint8_t UART_shoot_trigger( void );
uint8_t UART_get_shoot_level( void );

void UART_set_tx_command( uint8_t cmd );
void UART_set_tx_input( uint8_t input );
void UART_set_tx_adc( uint8_t adc_h, uint8_t adc_l );

void UART_Send_Packet();



ISR( USARTC0_TXC_vect );


#endif /* UART_H_ */