/*
 *	file		: global.h
 *  System		: Turtle 5K I/O Board
 *  Version		: V01.00
 *	Date		: 02-07-2013
 *  Description	: defines global variables
 *
 *  Author		: Dirk-Jan Vethaak
 *
 *	History		:
 *
 */


#include <stdbool.h>


#ifndef GLOBAL_H_
#define GLOBAL_H_

// Version info
// Version nr Vab.xy
#define C_VERSION_MAIOR_1 	   '0'		// a
#define C_VERSION_MAIOR_2      '1'		// b
#define C_VERSION_MINOR		   '2'		// x
#define C_VERSION_MIN		   '3'		// y

// F_CPU=16000000 in Project file

// PIN MASKS PORTE

#define SHOOT_TRIGGER_PM 0x01  // trigger pin PE0
#define SHOOT_ENABLE_PM 0x20  // enable pin PE5
#define DIRECTION_PM 0x08		// Direction output pin PE3


#define RS422_TX_ENABLE_PM 0x10

// FIXED hardware pin
#define PWM_PM 0x10			// PWM output pin PE4




// Fixed command values from host
#define CMD_SET_IO						0x01
#define CMD_SHOOT						0x02
#define CMD_READ_ADC_BALL_RIGHT_SINE	0x03
#define CMD_READ_ADC_BALL_LEFT_SINE		0x04
#define CMD_READ_ADC_BATTERY			0x05
#define CMD_READ_ADC_SHOOT				0x06
#define CMD_READ_COMPASS				0x07
#define CMD_HOME_POSITION				0x08

// Fixed command values from slave
#define CMD_GET_IO						0x01
#define CMD_GET_ADC_BALL_RIGHT_SINE		0x02
#define CMD_GET_ADC_BALL_LEFT_SINE		0x03
#define CMD_GET_ADC_BATTERY				0x04
#define CMD_GET_ADC_SHOOT				0x05
#if false // Compass not used
#define CMD_GET_MAG_X					0x06
#define CMD_GET_MAG_Y					0x07
#define CMD_GET_MAG_Z 0x08
#define CMD_GET_ACC_X 0x09
#define CMD_GET_ACC_Y 0x0A
#define CMD_GET_ACC_Z 0x0B
#endif




#define Local static;




#endif /* GLOBAL_H_ */