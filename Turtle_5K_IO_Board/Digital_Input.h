/*
 *	Module		: Digital Input.h
 *  System		: Turtle 5K I/O Board
 *  Version		: V01.00
 *	Date		: 02-07-2013
 *  Description	: Controls and read the digital Input data pins.
 *  Author		: Dirk-Jan Vethaak
 *
 *	History		:
 *
 */


#ifndef DIGITAL_INPUT_H_
#define DIGITAL_INPUT_H_

void Input_Init();


uint8_t Input_get();

uint8_t Input_Address_Get();

bool Input_emergency_active( void );


#endif /* DIGITAL_INPUT_H_ */