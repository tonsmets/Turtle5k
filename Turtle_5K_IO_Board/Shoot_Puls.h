/*
 *	Module		: Shoot_Puls.h
 *  System		: Turtle 5K I/O Board
 *  Version		: V01.00
 *	Date		: 02-07-2013
 *  Description	: Controls the trigger puls for the Shoot module in the Turtle 5K for a specific time.
 *				  0- 30 ms
 *  Author		: Dirk-Jan Vethaak
 *
 *	History		:
 *
 */

#ifndef SHOOT_PULSE_H_
#define SHOOT_PULSE_H_

// init the shoot puls module
void Shoot_Puls_Init();

// Triggers the shoot puls module to shoot
void Shoot_Trigger( uint8_t time );

bool Shoot_is_charging( void );

void Shoot_Pulse_emergency_stop( void );

#endif // SHOOT_PULSE_H_