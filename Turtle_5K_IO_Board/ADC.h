
/*
 *	Module		: ADC.h
 *  System		: Turtle 5K I/O Board
 *  Version		: V0.2
 *	Date		: 02-07-2013
 *  Description	: Controls the ADC for reading the analog input
 *  Author		: Dirk-Jan Vethaak
 *
 *	History		:
 *
 */


#ifndef ADC_H_
#define ADC_H_

//! \brief ADC reference settings
enum adc_ref_ch {
    //! Ball handling right hall sensor sine value.
    CH_BALL_RIGHT_SINE,
    //! Ball handling left hall sensor sine value.
    CH_BALL_LEFT_SINE,
    //! Battery ADC value
    CH_BATTERY,
    //! Shoot ADC value
    CH_SHOOT,
};


void ADC_init( void );

void adc_read_channels( void );

void ADC_GET( uint8_t channel, uint8_t* p_data );




#endif /* ADC_H_ */