/*
 *	Module		: Compass.c
 *  System		: Turtle 5K I/O Board
 *  Version		: V01.00
 *	Date		: 16-07-2013
 *  Description	: Controls and read the compass by I2C.
 *  Author		: Dirk-Jan Vethaak
 *
 *	History		:
 *
 */

#include <avr/io.h>
#include <stdbool.h>

#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "global.h"


/*
 *	Function	: TWI_MASTER_Init
 *  Author		: Dirk-Jan Vethaak
 *  Parameters	: none
 *	Return		: none
 *  Description	: Initialisation of TWI (I2C) Interface
 *
 *	History		:
 *
 */
void TWI_Master_Init()
{
    TWIC.MASTER.BAUD = 0x75; // 100Khz
    TWIC.MASTER.CTRLA = TWI_MASTER_ENABLE_bm | TWI_MASTER_INTLVL_HI_gc | TWI_MASTER_RIEN_bm | TWI_MASTER_WIEN_bm;
    TWIC.MASTER.STATUS = TWI_MASTER_BUSSTATE_IDLE_gc;

    PORTC.DIR &= 0xFC;
    PORTCFG.MPCMASK = 0x03;  // config as opendrain
}

/*
 *	Function	: TWI_MASTER_Start
 *  Author		: Dirk-Jan Vethaak
 *  Parameters	: Address byte
 *	Return		: int fout code -1 0
 *  Description	: Start TWI bus and send address
 *
 *	History		:
 *
 */
int TWI_MASTER_Start( uint8_t bByte )
{
    TWIC.MASTER.ADDR = bByte ;

    while( !( TWIC.MASTER.STATUS & TWI_MASTER_WIF_bm ) ); // wait to byte is shifted out

    if( TWIC.MASTER.STATUS & TWI_MASTER_RXACK_bm )
    {
        return -1 ;								// return error code -1 if returned NACK
    }

    return 0;
}

/*
 *	Function	: TWI_MASTER_write_byte
 *  Author		: Dirk-Jan Vethaak
 *  Parameters	: data byte
 *	Return		: fout code -1 0
 *  Description	: write data on TWI
 *
 *	History		:
 *
 */
int TWI_MASTER_Write_Byte( uint8_t bByte )
{
    TWIC.MASTER.DATA = bByte;

    while( !( TWIC.MASTER.STATUS & TWI_MASTER_WIF_bm ) ); // wait to byte is shifted out

    if( TWIC.MASTER.STATUS & TWI_MASTER_RXACK_bm )
    {
        return -1 ;								// return error code -1 if returned NACK

    } else
    {
        return 0 ;
    }


}
/*
 *	Function	: TWI_MASTER_stop
 *  Author		: Dirk-Jan Vethaak
 *  Parameters	: boolean to send nack
 *	Return		: none
 *  Description	: Stop TWi data and send NACK if bool is set
 *
 *	History		:
 *
 */
void TWI_MASTER_Stop( bool NACK )
{
    TWIC.MASTER.CTRLC = 0x03 | ( NACK << 2 ); // stop TWI

}



/*
 *	Function	: TWI_MASTER_read byte
 *  Author		: Dirk-Jan Vethaak
 *  Parameters	: none
 *	Return		: data byte
 *  Description	: reads byte of TWI after sending start, address, restart and register address
 *
 *	History		:
 *
 */
uint8_t TWI_Master_Read_Byte( uint8_t sla, uint8_t adres ) {

    TWIC.MASTER.ADDR = sla ;

    while( !( TWIC.MASTER.STATUS & TWI_MASTER_WIF_bm ) ); // wait to byte is shifted out

    if( TWIC.MASTER.STATUS & TWI_MASTER_RXACK_bm )
    {
        return -1 ;								// return error code -1 if returned NACK
    }

    TWIC.MASTER.DATA = adres;

    while( !( TWIC.MASTER.STATUS & TWI_MASTER_WIF_bm ) ); // wait to byte is shifted out

    if( TWIC.MASTER.STATUS & TWI_MASTER_RXACK_bm )
    {
        return -1 ;								// return error code -1 if returned NACK
    }

    TWIC.MASTER.ADDR = sla + 1 ;

    while( !( TWIC.MASTER.STATUS & TWI_MASTER_RIF_bm ) ); // wait until data is shifted in

    TWIC.MASTER.CTRLC = 0x07;

    return TWIC.MASTER.DATA;

}




/*
 *	Function	: Compass_init
 *  Author		: Dirk-Jan Vethaak
 *  Parameters	: none
 *	Return		: none
 *  Description	: Initialization of the compass module
 *
 *	History		:
 *
 */
void Compass_Init()
{


    //enable accelerometer
    TWI_MASTER_Start( 0x30 );
    // write acc
    TWI_MASTER_Write_Byte( 0x20 ); // CTRL_REG1_A
    TWI_MASTER_Write_Byte( 0x27 ); // normal power mode, 50 Hz data rate, all axes enabled
    TWI_MASTER_Stop( 0 );

    //enable magnetometer

    // write mag
    TWI_MASTER_Start( 0x3C );
    // write acc
    TWI_MASTER_Write_Byte( 0x02 ); // MR_REG_M
    TWI_MASTER_Write_Byte( 0x00 ); // continuous conversion mode
    TWI_MASTER_Stop( 0 );



}

