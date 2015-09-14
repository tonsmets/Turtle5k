
#ifndef COMPILER_AVR_H
#define COMPILER_AVR_H

#ifndef F_CPU

#define F_CPU 16000000UL
#endif

//#include <stdint.h>
#include <stdbool.h>
//#include <stdlib.h>
//#include "touch_api.h"

#define AVR_ENTER_CRITICAL_REGION( ) uint8_t volatile saved_sreg = SREG; \
                                     cli();

#define AVR_LEAVE_CRITICAL_REGION( ) SREG = saved_sreg;

#if defined( __ICCAVR__ )

#include <inavr.h>
#include <ioavr.h>
#include <intrinsics.h>
#include <pgmspace.h>

#ifndef __HAS_ELPM__
#define _MEMATTR  __flash
#else /* __HAS_ELPM__ */
#define _MEMATTR  __farflash
#endif /* __HAS_ELPM__ */

#define delay_us( us )   ( __delay_cycles( ( F_CPU / 1000000UL ) * ( us ) ) )

#define PRAGMA(x) _Pragma( #x )
#define ISR(vec) PRAGMA( vector=vec ) __interrupt void handler_##vec(void)
#define sei( ) (__enable_interrupt( ))
#define cli( ) (__disable_interrupt( ))

#define nop( ) (__no_operation())

#define watchdog_reset( ) (__watchdog_reset( ))


#define INLINE PRAGMA( inline=forced ) static

#define FLASH_DECLARE(x) _MEMATTR x
#define FLASH_STRING(x) ((_MEMATTR const char *)(x))
#define FLASH_STRING_T  char const _MEMATTR *
#define FLASH_BYTE_ARRAY_T uint8_t const _MEMATTR *
#define PGM_READ_BYTE(x) *(x)
#define PGM_READ_WORD(x) *(x)

#define SHORTENUM
#elif defined( __GNUC__ )

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#define delay_us( us )   (_delay_us( us ))

#define __delay_cycles __builtin_avr_delay_cycles

#define INLINE static inline

#define nop()   do { __asm__ __volatile__ ("nop"); } while (0)

#define MAIN_TASK_PROLOGUE int


#define MAIN_TASK_EPILOGUE() return -1;

#define SHORTENUM __attribute__ ((packed))

#else
#error Compiler not supported.
#endif

#endif