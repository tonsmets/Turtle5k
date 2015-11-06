
/*
 *	Module		: ADC.c
 *  System		: Turtle 5K I/O Board
 *  Version		: V0.2
 *	Date		: 02-07-2013
 *  Description	: Controls the ADC for reading the analog input
 *  Author		: Dirk-Jan Vethaak
 *
 *	History		:
 *
 */

#include <avr/io.h>
#include <util/delay.h>

// needed for adc calibration
#include <stddef.h>
#include <avr/pgmspace.h>
#include <stdbool.h>

#include "ADC.h"

static uint8_t ReadCalibrationByte( uint8_t );

/*! Sample storage (all four channels).*/
static int16_t adcSample[4];

/*
 *	Function	: ADC_Init
 *  Author		: Dennis Wokke
 *  Parameters	: none
 *	Return		: none
 *  Description	: Initialization of the ADC peripheral to measured a analog signal. ref voltage is the internal 1V.
 *				  All is done on interrupt basis.
 *
 *	History		:
 *
 */
/*
 *	Function	: ADC_Init
 *  Author		: Dirk-Jan Vethaak
 *  Parameters	: none
 *	Return		: none
 *  Description	: Initialisation of the ADC peripheral to measured a analog signal. ref voltage is the internal 1V.
 *
 *	History		:
 *
 */
void ADC_init()
{

    ADCA.CALL = ReadCalibrationByte( offsetof( NVM_PROD_SIGNATURES_t, ADCACAL0 ) );
    ADCA.CALH = ReadCalibrationByte( offsetof( NVM_PROD_SIGNATURES_t, ADCACAL1 ) );

    PORTA.DIR = 0x00 ; // set pinb 0-7 as input
    ADCA.CTRLB = ADC_RESOLUTION_12BIT_gc;

    ADCA.CTRLA |= 0x01; // enable ADC port A
    //ADCA.CTRLB= 0x08;	// unsigned , free running, 12 bit right adjusted
    ADCA.CTRLB = ADC_RESOLUTION_12BIT_gc;

    ADCA.REFCTRL = ADC_REFSEL_AREFA_gc ;   // use external 2.5 v op portb ref
    //ADCA.REFCTRL = ADC_REFSEL_AREFB_gc;

    //  ADCA.EVCTRL = 0xC0 ;  // sweep all channels no event
    //can be zero ?
    ADCA.PRESCALER = ADC_PRESCALER_DIV8_gc; // prescaler = 8

    ADCA.INTFLAGS = 0x0F; // clear all adc channels interrrupts


    //channels (all channels sweep in running mode )
    ADCA.CH0.CTRL =  ADC_CH_INPUTMODE_SINGLEENDED_gc; // single ended positive input, no start ..
    ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN1_gc; //select PA1 as ch0 input.

    ADCA.CH1.CTRL =  ADC_CH_INPUTMODE_SINGLEENDED_gc; // single ended positive input, no start ..
    ADCA.CH1.MUXCTRL = ADC_CH_MUXPOS_PIN2_gc; //select PA2 as ch1 input.

    ADCA.CH2.CTRL =  ADC_CH_INPUTMODE_SINGLEENDED_gc; // single ended positive input, no start ..
    ADCA.CH2.MUXCTRL = ADC_CH_MUXPOS_PIN3_gc; //select PA3 as ch2 input.

    ADCA.CH3.CTRL =  ADC_CH_INPUTMODE_SINGLEENDED_gc; // single ended positive input, no start ..
    ADCA.CH3.MUXCTRL = ADC_CH_MUXPOS_PIN4_gc; //select PA4 as ch3 input.

    _delay_ms( 5 ); // delay ADC start up

    // Start reading ADC immediately
    ADCA.CH0.CTRL |= ADC_CH_START_bm; // single ended positive input+start read from adc
    // Start reading ADC immediately
    ADCA.CH1.CTRL |= ADC_CH_START_bm; // single ended positive input+start read from adc
    // Start reading ADC immediately
    ADCA.CH2.CTRL |= ADC_CH_START_bm; // single ended positive input+start read from adc
    // Start reading ADC immediately
    ADCA.CH3.CTRL |= ADC_CH_START_bm; // single ended positive input+start read from adc

}

/*
 *	Function	: adc_read_channels
 *  Author		: Dennis Wokke
 *  Parameters	: none
 *	Return		: none
 *  Description	: Reads the ADC value of the four used channels and starts a new measurement
 *
 *	History		:
 *
 */
/*
 *	Function	: ADC_Init
 *  Author		: Dirk-Jan Vethaak
 *  Parameters	: none
 *	Return		: none
 *  Description	: Initialisation of the ADC peripheral to measured a analog signal. ref voltage is the internal 1V.
 *
 *	History		:
 *
 */
void adc_read_channels( void )
{
    if( ADCA.CH0.INTFLAGS )
    {
        adcSample[0] = ADCA.CH0RES;
        ADCA.CH0.INTFLAGS = 0x01;
        ADCA.CH0.CTRL |= ADC_CH_START_bm; // single ended positive input+start read from adc
    }

    if( ADCA.CH1.INTFLAGS )
    {
        adcSample[1] = ADCA.CH1RES;
        ADCA.CH1.INTFLAGS = 0x01;
        ADCA.CH1.CTRL |= ADC_CH_START_bm; // single ended positive input+start read from adc
    }

    if( ADCA.CH2.INTFLAGS )
    {
        adcSample[2] = ADCA.CH2RES;
        ADCA.CH2.INTFLAGS = 0x01;
        ADCA.CH2.CTRL |= ADC_CH_START_bm; // single ended positive input+start read from adc
    }

    if( ADCA.CH3.INTFLAGS )
    {
        adcSample[3] = ADCA.CH3RES;
        ADCA.CH3.INTFLAGS = 0x01;
        ADCA.CH3.CTRL |= ADC_CH_START_bm; // single ended positive input+start read from adc
    }

    return;
}

/*
 *	Function	: ADC_GET
 *  Author		: Dirk-Jan Vethaak
 *  Parameters	: none
 *	Return		: none
 *  Description	: Start ADC conversion.
 *				  Data is read in Main  by abBuffer_tx[3] = ADCA.CH0RESH;
 *										   abBuffer_tx[4] = ADCA.CH0RESL;
 *
 *	History		:
 *
 */
void ADC_GET( uint8_t channel, uint8_t* p_data )
{
    *p_data = ( adcSample[channel] >> 8 ) & 0x00FF;
    p_data++;
    *p_data = adcSample[channel] & 0x00FF;

    return;
}

static uint8_t ReadCalibrationByte( uint8_t index )
{
    uint8_t result;

    /* Load the NVM Command register to read the calibration row. */
    NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
    result = pgm_read_byte( index );

    /* Clean up NVM Command register. */
    NVM_CMD = NVM_CMD_NO_OPERATION_gc;

    return( result );
}