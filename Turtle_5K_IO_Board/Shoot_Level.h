/*
 *	Module		: Shoot_Level.h
 *  System		: Turtle 5K I/O Board
 *  Version		: V01.00
 *	Date		: 02-07-2013
 *  Description	: Controls the steppenmotor driver for the Shoot Level.
 *				  Using an PWM signal for a specific time.
 *  Author		: Dirk-Jan Vethaak
 *
 *	History		:
 *
 */

#ifndef SHOOT_LEVEL_H_
#define SHOOT_LEVEL_H_

void Shoot_Level_Init();
void Shoot_Level_Set( uint8_t bPosition );
void Shoot_Level_go_home( void );
void Shoot_Level_do_step( void );
void Shoot_Level_emergency_stop( void );
ISR( TCF0_OVF_vect );



#endif /* SHOOT_LEVEL_H_ */