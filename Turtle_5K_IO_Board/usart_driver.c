
/* This file has been prepared for Doxygen automatic documentation generation.*/
#include "usart_driver.h"



void USART_InterruptDriver_Initialize( USART_data_t * usart_data,
                                       USART_t * usart,
                                       USART_DREINTLVL_t dreIntLevel )
{
    usart_data->usart = usart;
    usart_data->dreIntLevel = dreIntLevel;

    usart_data->buffer.RX_Tail = 0;
    usart_data->buffer.RX_Head = 0;
    usart_data->buffer.TX_Tail = 0;
    usart_data->buffer.TX_Head = 0;
}


void USART_InterruptDriver_DreInterruptLevel_Set( USART_data_t * usart_data,
        USART_DREINTLVL_t dreIntLevel )
{
    usart_data->dreIntLevel = dreIntLevel;
}


bool USART_TXBuffer_FreeSpace( USART_data_t * usart_data )
{
    /* Make copies to make sure that volatile access is specified. */
    uint8_t tempHead = ( usart_data->buffer.TX_Head + 1 ) & USART_TX_BUFFER_MASK;
    uint8_t tempTail = usart_data->buffer.TX_Tail;

    /* There are data left in the buffer unless Head and Tail are equal. */
    return ( tempHead != tempTail );
}



bool USART_TXBuffer_PutByte( USART_data_t * usart_data, uint8_t data )
{
    uint8_t tempCTRLA;
    uint8_t tempTX_Head;
    bool TXBuffer_FreeSpace;
    USART_Buffer_t * TXbufPtr;

    TXbufPtr = &usart_data->buffer;
    TXBuffer_FreeSpace = USART_TXBuffer_FreeSpace( usart_data );


    if( TXBuffer_FreeSpace )
    {
        tempTX_Head = TXbufPtr->TX_Head;
        TXbufPtr->TX[tempTX_Head] = data;
        /* Advance buffer head. */
        TXbufPtr->TX_Head = ( tempTX_Head + 1 ) & USART_TX_BUFFER_MASK;

        /* Enable DRE interrupt. */
        tempCTRLA = usart_data->usart->CTRLA;
        tempCTRLA = ( tempCTRLA & ~USART_DREINTLVL_gm ) | usart_data->dreIntLevel;
        usart_data->usart->CTRLA = tempCTRLA;
    }

    return TXBuffer_FreeSpace;
}



bool USART_RXBufferData_Available( USART_data_t * usart_data )
{
    /* Make copies to make sure that volatile access is specified. */
    uint8_t tempHead = usart_data->buffer.RX_Head;
    uint8_t tempTail = usart_data->buffer.RX_Tail;

    /* There are data left in the buffer unless Head and Tail are equal. */
    return ( tempHead != tempTail );
}



uint8_t USART_RXBuffer_GetByte( USART_data_t * usart_data )
{
    USART_Buffer_t * bufPtr;
    uint8_t ans;

    bufPtr = &usart_data->buffer;
    ans = ( bufPtr->RX[bufPtr->RX_Tail] );

    /* Advance buffer tail. */
    bufPtr->RX_Tail = ( bufPtr->RX_Tail + 1 ) & USART_RX_BUFFER_MASK;

    return ans;
}



bool USART_RXComplete( USART_data_t * usart_data )
{
    USART_Buffer_t * bufPtr;
    bool ans;

    bufPtr = &usart_data->buffer;
    /* Advance buffer head. */
    uint8_t tempRX_Head = ( bufPtr->RX_Head + 1 ) & USART_RX_BUFFER_MASK;

    /* Check for overflow. */
    uint8_t tempRX_Tail = bufPtr->RX_Tail;
    uint8_t data = usart_data->usart->DATA;

    if( tempRX_Head == tempRX_Tail ) {
        ans = false;

    } else {
        ans = true;
        usart_data->buffer.RX[usart_data->buffer.RX_Head] = data;
        usart_data->buffer.RX_Head = tempRX_Head;
    }

    return ans;
}



void USART_DataRegEmpty( USART_data_t * usart_data )
{
    USART_Buffer_t * bufPtr;
    bufPtr = &usart_data->buffer;

    /* Check if all data is transmitted. */
    uint8_t tempTX_Tail = usart_data->buffer.TX_Tail;

    if( bufPtr->TX_Head == tempTX_Tail ) {
        /* Disable DRE interrupts. */
        uint8_t tempCTRLA = usart_data->usart->CTRLA;
        tempCTRLA = ( tempCTRLA & ~USART_DREINTLVL_gm ) | USART_DREINTLVL_OFF_gc;
        usart_data->usart->CTRLA = tempCTRLA;

    } else {
        /* Start transmitting. */
        uint8_t data = bufPtr->TX[usart_data->buffer.TX_Tail];
        usart_data->usart->DATA = data;

        /* Advance buffer tail. */
        bufPtr->TX_Tail = ( bufPtr->TX_Tail + 1 ) & USART_TX_BUFFER_MASK;
    }
}


void USART_NineBits_PutChar( USART_t * usart, uint16_t data )
{
    if( data & 0x0100 ) {
        usart->CTRLB |= USART_TXB8_bm;

    } else {
        usart->CTRLB &= ~USART_TXB8_bm;
    }

    usart->DATA = ( data & 0x00FF );
}


uint16_t USART_NineBits_GetChar( USART_t * usart )
{
    if( usart->CTRLB & USART_RXB8_bm ) {
        return( 0x0100 | usart->DATA );

    } else {
        return( usart->DATA );
    }
}
