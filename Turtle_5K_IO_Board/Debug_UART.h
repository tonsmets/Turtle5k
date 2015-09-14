/*
 *	Module		: Debug_UART.h
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


#ifndef DEBUG_UART_H_
#define DEBUG_UART_H_

void initUSARTD0( void );

void putchUSARTD0( char cx );

void putsUSARTD0( uint8_t *ptr );

void debug_comm_data_received( void );


#endif /* DEBUG_UART_H_ */