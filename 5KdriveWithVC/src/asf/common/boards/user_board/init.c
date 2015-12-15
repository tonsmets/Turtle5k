/**
 * \file
 *
 * \brief User board initialization template
 *
 */

#include <asf.h>
#include <board.h>
#include <conf_board.h>
#include <avr/io.h>

void board_init(void)
{
	/* This function is meant to contain board-specific initialization code
	 * for, e.g., the I/O pins. The initialization can rely on application-
	 * specific board configuration, found in conf_board.h.
	 */
	pmic_init();
	sysclk_init();
	ioport_init();
	
	ioport_set_pin_dir(LED0, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(LED1, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(ENC_A, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(ENC_B, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(ENC_I, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(DRV_EN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(PWM_A, IOPORT_DIR_OUTPUT);
//	ioport_set_pin_mode(PWM_A, IOPORT_MODE_INVERT_PIN);
	ioport_set_pin_dir(PWM_B, IOPORT_DIR_OUTPUT);
//	ioport_set_pin_mode(PWM_B, IOPORT_MODE_INVERT_PIN);
	ioport_set_pin_dir(RX1, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(TX1, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(RS422DE, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(RS422RE, IOPORT_DIR_OUTPUT);
//	ioport_set_pin_mode(RS422RE, IOPORT_MODE_INVERT_PIN);
	ioport_set_pin_dir(DRV_CS, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(DRV_MOSI, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(DRV_MISO, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(DRV_MISO, IOPORT_MODE_PULLUP);	
	ioport_set_pin_dir(DRV_SCLK, IOPORT_DIR_OUTPUT);
//	ioport_set_pin_dir(MY_BUTTON, IOPORT_DIR_INPUT);
//	ioport_set_pin_mode(MY_BUTTON, IOPORT_MODE_INVERT_PIN);

}
