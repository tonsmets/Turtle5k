/*
 * Compass.h
 *
 * Created: 8-7-2013 14:54:21
 *  Author: dirkjan
 */


// NOT IMPLEMENTED

#ifndef COMPASS_H_
#define COMPASS_H_




void TWI_Master_Init();


int TWI_MASTER_Start( uint8_t bByte );


int TWI_MASTER_Write_Byte( uint8_t bByte );


void TWI_MASTER_Stop( bool NACK );

char readByteTWI( uint8_t sla, uint8_t adres );

uint8_t TWI_MASTER_Read_Byte();

void read_data_raw();


void Compass_Init();



#endif /* COMPASS_H_ */