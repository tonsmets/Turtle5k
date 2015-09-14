/* This file has been prepared for Doxygen automatic documentation generation.*/
#ifndef USART_DRIVER_H
#define USART_DRIVER_H

#include "avr_compiler.h"

/* USART buffer defines. */

/* \brief  Receive buffer size: 2,4,8,16,32,64,128 or 256 bytes. */
#define USART_RX_BUFFER_SIZE 4
/* \brief Transmit buffer size: 2,4,8,16,32,64,128 or 256 bytes */
#define USART_TX_BUFFER_SIZE 64
/* \brief Receive buffer mask. */
#define USART_RX_BUFFER_MASK ( USART_RX_BUFFER_SIZE - 1 )
/* \brief Transmit buffer mask. */
#define USART_TX_BUFFER_MASK ( USART_TX_BUFFER_SIZE - 1 )


#if ( USART_RX_BUFFER_SIZE & USART_RX_BUFFER_MASK )
#error RX buffer size is not a power of 2
#endif
#if ( USART_TX_BUFFER_SIZE & USART_TX_BUFFER_MASK )
#error TX buffer size is not a power of 2
#endif


/* \brief USART transmit and receive ring buffer. */
typedef struct USART_Buffer
{
    /* \brief Receive buffer. */
    volatile uint8_t RX[USART_RX_BUFFER_SIZE];
    /* \brief Transmit buffer. */
    volatile uint8_t TX[USART_TX_BUFFER_SIZE];
    /* \brief Receive buffer head. */
    volatile uint8_t RX_Head;
    /* \brief Receive buffer tail. */
    volatile uint8_t RX_Tail;
    /* \brief Transmit buffer head. */
    volatile uint8_t TX_Head;
    /* \brief Transmit buffer tail. */
    volatile uint8_t TX_Tail;
} USART_Buffer_t;


typedef struct Usart_and_buffer
{
    /* \brief Pointer to USART module to use. */
    USART_t * usart;
    /* \brief Data register empty interrupt level. */
    USART_DREINTLVL_t dreIntLevel;
    /* \brief Data buffer. */
    USART_Buffer_t buffer;
} USART_data_t;


/* Macros. */

#define USART_Format_Set(_usart, _charSize, _parityMode, _twoStopBits)         \
        (_usart)->CTRLC = (uint8_t) _charSize | _parityMode |                      \
                          (_twoStopBits ? USART_SBMODE_bm : 0)


#define USART_Baudrate_Set(_usart, _bselValue, _bScaleFactor)                  \
        (_usart)->BAUDCTRLA =(uint8_t)_bselValue;                                           \
        (_usart)->BAUDCTRLB =(_bScaleFactor << USART_BSCALE0_bp)|(_bselValue >> 8)


#define USART_Rx_Enable(_usart) ((_usart)->CTRLB |= USART_RXEN_bm)


#define USART_Rx_Disable(_usart) ((_usart)->CTRLB &= ~USART_RXEN_bm)


#define USART_Tx_Enable(_usart) ((_usart)->CTRLB |= USART_TXEN_bm)


#define USART_Tx_Disable(_usart) ((_usart)->CTRLB &= ~USART_TXEN_bm)


#define USART_RxdInterruptLevel_Set(_usart, _rxdIntLevel)                      \
        ((_usart)->CTRLA = ((_usart)->CTRLA & ~USART_RXCINTLVL_gm) | _rxdIntLevel)


#define USART_TxdInterruptLevel_Set(_usart, _txdIntLevel)                      \
        (_usart)->CTRLA = ((_usart)->CTRLA & ~USART_TXCINTLVL_gm) | _txdIntLevel



#define USART_DreInterruptLevel_Set(_usart, _dreIntLevel)                      \
        (_usart)->CTRLA = ((_usart)->CTRLA & ~USART_DREINTLVL_gm) | _dreIntLevel


#define USART_SetMode(_usart, _usartMode)                                      \
        ((_usart)->CTRLC = ((_usart)->CTRLC & (~USART_CMODE_gm)) | _usartMode)



#define USART_IsTXDataRegisterEmpty(_usart) (((_usart)->STATUS & USART_DREIF_bm) != 0)



#define USART_PutChar(_usart, _data) ((_usart)->DATA = _data)



#define USART_IsRXComplete(_usart) (((_usart)->STATUS & USART_RXCIF_bm) != 0)




#define USART_GetChar(_usart)  ((_usart)->DATA)


/* Functions for interrupt driven driver. */
void USART_InterruptDriver_Initialize( USART_data_t * usart_data,
                                       USART_t * usart,
                                       USART_DREINTLVL_t dreIntLevel );

void USART_InterruptDriver_DreInterruptLevel_Set( USART_data_t * usart_data,
        USART_DREINTLVL_t dreIntLevel );

bool USART_TXBuffer_FreeSpace( USART_data_t * usart_data );
bool USART_TXBuffer_PutByte( USART_data_t * usart_data, uint8_t data );
bool USART_RXBufferData_Available( USART_data_t * usart_data );
uint8_t USART_RXBuffer_GetByte( USART_data_t * usart_data );
bool USART_RXComplete( USART_data_t * usart_data );
void USART_DataRegEmpty( USART_data_t * usart_data );

/* Functions for polled driver. */
void USART_NineBits_PutChar( USART_t * usart, uint16_t data );
uint16_t USART_NineBits_GetChar( USART_t * usart );

#endif
