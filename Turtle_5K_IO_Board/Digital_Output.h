/*
 *	Module		: Output.h
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


#ifndef DIGITAL_OUTPUT_H_
#define DIGITAL_OUTPUT_H_

void Output_Init();
void Output_Set( const uint8_t bData );
void Output_all_off( void );

#endif /* DIGITAL_OUTPUT_H_ */