/**
 * \file
 *
 * \brief User board configuration template
 *
 */

#ifndef CONF_BOARD_H
#define CONF_BOARD_H

#define LED0		IOPORT_CREATE_PIN(PORTF, 4)
#define LED1		IOPORT_CREATE_PIN(PORTF, 5)
#define ENC_A		IOPORT_CREATE_PIN(PORTE, 0)
#define ENC_B		IOPORT_CREATE_PIN(PORTE, 1)
#define ENC_I		IOPORT_CREATE_PIN(PORTE, 2)
#define RS422DE		IOPORT_CREATE_PIN(PORTE, 6)
#define RS422RE		IOPORT_CREATE_PIN(PORTE, 7)
#define RX1			IOPORT_CREATE_PIN(PORTF, 2)
#define TX1			IOPORT_CREATE_PIN(PORTF, 3)
#define DRV_EN		IOPORT_CREATE_PIN(PORTF, 1)
#define DRV_CS		IOPORT_CREATE_PIN(PORTC, 4)
#define DRV_MOSI	IOPORT_CREATE_PIN(PORTC, 5)
#define DRV_MISO	IOPORT_CREATE_PIN(PORTC, 6)
#define DRV_SCLK	IOPORT_CREATE_PIN(PORTC, 7)
#define PWM_A		IOPORT_CREATE_PIN(PORTD, 2)
#define PWM_B		IOPORT_CREATE_PIN(PORTD, 3)


#endif // CONF_BOARD_H
