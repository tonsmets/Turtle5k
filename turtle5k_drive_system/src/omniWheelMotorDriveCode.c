/**
 * \file
 *
 * \brief 5K Driver
 * Version  : 1.1
 * Date		: 10-03-2014
 * Author	: Arthur Ketels	
 *
 * Changelog
 * 0.1 -> 0.2 : Optimize integral windup
 *				Add ntc2 channel
 *				set bit2 of cmd will swap encoder for ntc2 result
 *
 * 0.2 -> 0.3 : Change ADC timing in PWMtop()
 *
 * 0.3 -> 0.4 : Handle hardware disable correctly
 *				ADC current zero calibration is done when DRV enable is confirmed
 *				Added internal pull-up on MISO
 *				Added undervoltage lockout (set at 10.5V)
 *
 * 0.4 -> 1.0 : Add RS422 DE RE pins and set default to RS422 mode
 *				Select pin A7 as NTC2 channel, also used for tacho
 *				Compatible with v1.2 board
 *
 * 1.0 -> 1.1 : Change current sampling point to shift togeter with motor PWM
 *				This prevents discontinuities around old 50% sample point.
 
 
* 1.0.1  dj  : edit DRVreg[4] = 0;  to DRVreg[3] = 0; has index 0-3 instead of 0-4
 
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
#include <asf.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
//#include <time.h>

#define PWMFREQ			40000
#define PWMPER			(BOARD_PLL_HZ * 2 / PWMFREQ)
#define SERVICEFREQ		1000
#define SERVICEPER		(BOARD_PLL_HZ / (SERVICEFREQ * 4))
#define SUPERHIGHRES	0x04
#define DRVSPI			SPIC
#define SER1			USARTF0
#define ADC				ADCA


#define WHEEL_RADIUS	0.1016

#define SOF 0x5a

#define	RESISTOR	0.001
#define DRVGAIN		10.0
#define I2U			RESISTOR * DRVGAIN
#define U2BIT		(1024.0 / 1.25)
#define I2RGAIN		(I2U * U2BIT * 4.0 * 0.01 * 256)
#define R2IGAIN		(256.0 / (I2U * U2BIT * 4.0 * 0.01))
#define CURRENTLIM	100.0
#define ILIMRAW		(I2U * U2BIT * 2.0 * CURRENTLIM)


#define bmENABLE	0x01
#define bmMODE		0x06
#define bmMODEA		0x00
#define bmMODEC		0x02
#define bmMODEV		0x04

#define MAXCONTROL	(int16_t)(PWMPER * 0.96)
#define MINCONTROL	(int16_t)(PWMPER * 0.02)
#define HALFPWM		(int16_t)(PWMPER * 0.5)

// PI servo control constants
#define PGAIN		(5.0 * 256)
#define IGAIN		(12.0 * 256)

// Unity gain raw line voltage
#define UPWMGAIN	1800
#define UPWMVOLT	24.0	
// Minimum raw line voltage for gain calculation
#define LPWMGAIN	600
#define SHUTDOWNVOLTAGE (10.5 * UPWMGAIN / UPWMVOLT)
#define STARTUPVOLTAGE	(11.0 * UPWMGAIN / UPWMVOLT)

// 100 = 100ms
#define WATCHDOG	100
// temp 85C
#define OVERTEMP	240

#define DRV3REG		((0<<2) | 0x01)


typedef struct
{
	uint8_t		sof;
	uint8_t		type;
	uint8_t		cmd;
	uint16_t	data1;
	uint16_t	data2;
	uint8_t		crc;
} packett;

uint8_t inpacket[8], outpacket[7];	
uint16_t tcnt, tcnt2;
uint16_t pwmv = PWMPER / 2;
uint8_t	pwmdir;
int16_t qei_raw, qei_prev, iqei_raw;
int16_t qei_val;
int16_t qei_delta;

int16_t iMeasured_value;
int16_t iDifference, iPrevious_value;
int16_t iError, iPreviousError, Ki, iIntegral, Kp, iControl;
int16_t iMeasured_velocity, setpoint;

int16_t relative_speed;	
int16_t setpoint_relative_speed;

int16_t i_err, i_con;
int16_t i_setpoint = 0;
int32_t i_i;
uint8_t cenable, senable, cmode, smode;

int16_t	ADCso1, ADCso2, ADCvpwr, ADCntc, ADCntc2;
int16_t cv_I, cv_U, conv_trig, I_meas, PWMgain;
int16_t so1_zero, so2_zero;
int16_t	cv_I2, cv_a, cv_b;
uint8_t controltick;


uint8_t wdcnt, ntccnt, overtemp;
uint8_t undervoltage;
uint8_t calstate;
uint8_t caldone;
uint16_t ledcnt;
uint16_t cal_a1, cal_a2;
int DRVstate;
uint16_t DRVwr;
uint16_t DRVreg[4];
int DRVadr;
uint16_t DRVraw;
uint16_t prevDRVwr;
uint8_t DRVt1, DRVt2, DRVr1, DRVr2;

#define bmDRVread      (1 << 15)
#define bmDRVwritw     (0 << 15)
#define bmDRVstat1     (0x0 << 11)
#define bmDRVstat2     (0x1 << 11)
#define bmDRVctrl1     (0x2 << 11)
#define bmDRVctrl2     (0x3 << 11)

uint8_t ifcnt;
int16_t ifiltsum, ifilt;
struct adc_channel_config adcch_conf;
uint8_t adctoggle;

uint8_t rcrc;

#define RX_BUFFER_SIZE_USARTF0 64
#define TX_BUFFER_SIZE_USARTF0 250
char rx_buffer_USARTF0[RX_BUFFER_SIZE_USARTF0];
volatile unsigned char rx_wr_index_USARTF0=0,rx_rd_index_USARTF0=0,rx_counter_USARTF0=0;
uint8_t rx_buffer_overflow_USARTF0=0;
char tx_buffer_USARTF0[TX_BUFFER_SIZE_USARTF0];
volatile unsigned char tx_wr_index_USARTF0=0,tx_rd_index_USARTF0=0,tx_counter_USARTF0=0;
int uart_getcharF0(FILE *stream);
int uart_putcharF0(char c);
static FILE mystdout = FDEV_SETUP_STREAM (uart_putcharF0, NULL, _FDEV_SETUP_WRITE);
static FILE mystdin  = FDEV_SETUP_STREAM (NULL, uart_getcharF0, _FDEV_SETUP_READ);

// USARTF0 Receiver interrupt service routine
ISR(USARTF0_RXC_vect)
{
	unsigned char status;
	char data;

	status=USARTF0.STATUS;
	data=USARTF0.DATA;
	if ((status & (USART_FERR_bm | USART_PERR_bm | USART_BUFOVF_bm)) == 0)
	{
		rx_buffer_USARTF0[rx_wr_index_USARTF0++]=data;
		if (rx_wr_index_USARTF0 == RX_BUFFER_SIZE_USARTF0) rx_wr_index_USARTF0=0;
		if (++rx_counter_USARTF0 == RX_BUFFER_SIZE_USARTF0)
		{
			rx_counter_USARTF0=0;
			rx_buffer_overflow_USARTF0=1;
		};
	};
}

int uart_getcharF0(FILE *stream)
{
	char data;

	while (rx_counter_USARTF0==0);
	data=rx_buffer_USARTF0[rx_rd_index_USARTF0++];
	if (rx_rd_index_USARTF0 == RX_BUFFER_SIZE_USARTF0) rx_rd_index_USARTF0=0;
	--rx_counter_USARTF0;
	return data;
}

// USARTF0 Transmitter interrupt service routine
ISR(USARTF0_TXC_vect)
{
	if (tx_counter_USARTF0)
	{
		--tx_counter_USARTF0;
		USARTF0.DATA=tx_buffer_USARTF0[tx_rd_index_USARTF0++];
		if (tx_rd_index_USARTF0 == TX_BUFFER_SIZE_USARTF0) tx_rd_index_USARTF0=0;
	}
}

int uart_putcharF0(char c)
{
	while (tx_counter_USARTF0 == TX_BUFFER_SIZE_USARTF0);
	cli();
	if (tx_counter_USARTF0 || ((USARTF0.STATUS & USART_DREIF_bm)==0))
	{
		tx_buffer_USARTF0[tx_wr_index_USARTF0++]=c;
		if (tx_wr_index_USARTF0 == TX_BUFFER_SIZE_USARTF0) tx_wr_index_USARTF0=0;
		++tx_counter_USARTF0;
	}
	else
	USARTF0.DATA=c;
	sei();
	return 0;
}

static void set_stdinout(void)
{
	stdout = &mystdout;
	stdin = &mystdin;
}

/* 8 bit CRC , polynomal X^8+X^5+X^4+1 , start value = 0 */
static uint8_t crc8(uint8_t crc, uint8_t crc_data)
{
	uint8_t i;

	i = crc_data ^ crc;
	crc = 0;
	if(i & 0x01)
	crc ^= 0x5e;
	if(i & 0x02)
	crc ^= 0xbc;
	if(i & 0x04)
	crc ^= 0x61;
	if(i & 0x08)
	crc ^= 0xc2;
	if(i & 0x10)
	crc ^= 0x9d;
	if(i & 0x20)
	crc ^= 0x23;
	if(i & 0x40)
	crc ^= 0x46;
	if(i & 0x80)
	crc ^= 0x8c;

	return(crc);
}

/*static uint32_t hextoint(const char *hex)
{
	uint32_t result = 0;

	while (*hex)
	{
		if (*hex > 47 && *hex < 58)
		result += (*hex - 48);
		else if (*hex > 64 && *hex < 71)
		result += (*hex - 55);
		else if (*hex > 96 && *hex < 103)
		result += (*hex - 87);

		if (*++hex)
		result <<= 4;
	}

	return result;
}
*/
static int16_t q8mul(int16_t value, int16_t mul)
{
	int32_t res;
	res = ((int32_t)value * mul) >> 8;
	if (res > 32767)
	res = 32767;
	if (res < -32768)
	res = -32768;
	return res;
}

static int32_t q8mul32(int16_t value, int16_t mul)
{
	int32_t res;
	res = ((int32_t)value * mul) >> 8;
	return res;
}

static int16_t clip16(int16_t val, int16_t clipl, int16_t cliph)
{
	if(val < clipl)
	return clipl;
	else if(val > cliph)
	return cliph;
	return val;
}

/*static uint8_t in_window(int16_t val, int16_t wl, int16_t wh)
{
	if((val <= wh) && (val >= wl))
	return 1;
	else
	return 0;
}*/

static int32_t clip32(int32_t val, int32_t clipl, int32_t cliph)
{
	if(val < clipl)
	return clipl;
	else if(val > cliph)
	return cliph;
	return val;
}

static void servoloop(void)
{		
	// current control error
	i_err = i_setpoint - cv_I;
	// integrator, wind up limiter
	i_i = clip32(i_i + i_err, -240000, 240000);
	// PWMgain = 1 / line voltage, 1 at 24V ( = UPWMGAIN )
	if (ADCvpwr < LPWMGAIN)
		PWMgain = (UPWMGAIN / LPWMGAIN) * 256;
	else  
		PWMgain = (UPWMGAIN << 4)/(ADCvpwr >> 3);
	// PI controller : control = ((PGAIN * i_err) + (IGAIN * i_i)) * PWMgain
	// clipped at maximum duty cycle MAXCONTROL
	 i_con = (int16_t)clip32(q8mul32(q8mul32(i_err, PGAIN) + q8mul32((int16_t)(i_i >> 8), IGAIN), PWMgain), -MAXCONTROL, MAXCONTROL);
}	

static void inline write_conv_trig(uint16_t value)
{
	TCD0.CCA = value;
}

static void inline write_pwm_a(uint16_t value)
{
	TCD0.CCC = value;
}

static void inline write_pwm_b(uint16_t value)
{
	TCD0.CCD = value;
}

static void PWMtop(void)
{
	ADCso1 = adc_get_result(&ADC, ADC_CH0) - so1_zero;
	ADCso2 = adc_get_result(&ADC, ADC_CH1) - so2_zero;
	ADCvpwr = adc_get_result(&ADC, ADC_CH2);
	
	if(i_con >= 0)
	{
		I_meas = -ADCso2;
	}
	else
	{
		I_meas = ADCso1;
	}
	if(controltick++ > 0)
	{
		controltick = 0;
		qei_raw = tc_read_count(&TCE0);
		iqei_raw = qei_raw;
		if(iqei_raw > 0x8000)
		{
			iqei_raw = iqei_raw - 0x8000; 
		}
		qei_val = qei_raw - qei_prev;
		qei_prev = qei_raw;
		cv_I = (I_meas + cv_I2); // sample two cycles for servoloop
		servoloop();
		cv_a = 0;
		cv_b = 0;
		if(senable)
		{
			//forward
			if(i_con > 0)
			{
				cv_a = MINCONTROL + i_con;
				
				// conv_trig was old value of halfpwm
				conv_trig = (cv_a + MINCONTROL + MINCONTROL);
				cv_b = MINCONTROL;
			}
			//backwards
			else if(i_con < 0)
			{
				cv_a = MINCONTROL;
				cv_b = MINCONTROL - i_con;    // - - = + i_con
				
					// conv_trig was old value of halfpwm
				conv_trig = (cv_b + MINCONTROL + MINCONTROL);
			}
		}		
		write_pwm_a(cv_a);
		write_pwm_b(cv_b);		
		write_conv_trig(conv_trig);
		ifiltsum += I_meas;
		if(ifcnt++>31)
		{
			ifilt = ifiltsum >> 4;
			ifcnt = 0;
			ifiltsum = 0;
		}
	}
	else /// ????
	{
		write_conv_trig((conv_trig + (int16_t)(0.5 * PWMPER)) % PWMPER);		
	}
	cv_I2 = I_meas;

	// alternate ntc1 and ntc2 sensor read out
	if(!adctoggle)
	{
		adctoggle = 1;
		ADCntc = adc_get_result(&ADC, ADC_CH3);
		// prepare next scan for NTC2 sensor
		ADC.CH3.MUXCTRL = (ADCCH_POS_PIN7 << ADC_CH_MUXPOS_gp) | (ADCCH_NEG_PIN1 << ADC_CH_MUXNEG_gp);
	}
	else
	{
		adctoggle = 0;
		ADCntc2 = adc_get_result(&ADC, ADC_CH3);
		// prepare next scan for NTC1 sensor
		ADC.CH3.MUXCTRL = (ADCCH_POS_PIN5 << ADC_CH_MUXPOS_gp) | (ADCCH_NEG_PIN1 << ADC_CH_MUXNEG_gp);
	}
}

static void adc_init(void)
{
	struct adc_config adc_conf;

	sysclk_enable_module(SYSCLK_PORT_GEN, SYSCLK_EVSYS);
	//EVSYS.CH3MUX = EVSYS_CHMUX_TCD0_OVF_gc;
	EVSYS.CH3MUX = EVSYS_CHMUX_TCD0_CCA_gc;
	adc_read_configuration(&ADC, &adc_conf);
	adcch_read_configuration(&ADC, ADC_CH0, &adcch_conf);

	adc_set_conversion_parameters(&adc_conf, ADC_SIGN_ON , ADC_RES_12, ADC_REFSEL_AREFA_gc);
	/* set adc start at event ch 3, sweep over 4 channels */
	adc_set_conversion_trigger(&adc_conf, ADC_TRIG_EVENT_SWEEP, 4, 3);
	adc_set_clock_rate(&adc_conf, 2000000UL);
	adc_write_configuration(&ADC, &adc_conf);

	// current input 1
	adcch_set_input(&adcch_conf, ADCCH_POS_PIN2, ADCCH_NEG_PIN1, 1);
	adcch_write_configuration(&ADC, ADC_CH0, &adcch_conf);
	// current input 2
	adcch_set_input(&adcch_conf, ADCCH_POS_PIN3, ADCCH_NEG_PIN1, 1);
	adcch_write_configuration(&ADC, ADC_CH1, &adcch_conf);
	// PWM supply voltage
	adcch_set_input(&adcch_conf, ADCCH_POS_PIN4, ADCCH_NEG_PIN1, 1);
	adcch_write_configuration(&ADC, ADC_CH2, &adcch_conf);
	// NTC sensor
	adcch_set_input(&adcch_conf, ADCCH_POS_PIN5, ADCCH_NEG_PIN1, 1);
	adcch_write_configuration(&ADC, ADC_CH3, &adcch_conf);

	adc_enable(&ADC);
}

static void qei_init(void)
{
	PORTE.PIN0CTRL = PORT_ISC_LEVEL_gc;         // QDPH0  - D0
	PORTE.PIN1CTRL = PORT_ISC_LEVEL_gc;         // QDPH90 - D1	
	EVSYS.CH0MUX = EVSYS_CHMUX_PORTE_PIN0_gc;
	EVSYS_CH0CTRL = EVSYS_QDEN_bm | EVSYS_DIGFILT_2SAMPLES_gc; 
	tc_enable(&TCE0);
	tc_set_input_capture(&TCE0, TC_EVSEL_CH0_gc, TC_EVACT_QDEC_gc);
	tc_write_period(&TCE0, 0xffff);
	tc_write_clock_source(&TCE0, TC_CLKSEL_DIV1_gc);
}

static void drv_disable(void)
{
	i_i = 0;
	i_setpoint = 0;
	ioport_set_pin_level(LED0, false);
	senable = 0;	
}

static void DRV8301_sm(void)
{
	int tmpadr;
	uint16_t tdata;
	switch(DRVstate)
	{
		case 0:
			ioport_set_pin_level(DRV_CS, false);
			if(DRVadr < 0x04)
			{
				tdata = bmDRVread | (DRVadr << 11);
				DRVt1 = (uint8_t)tdata;
				DRVt2 = (uint8_t)(tdata >> 8);				
			}
			else
			{
				tdata = bmDRVread;
				DRVt1 = (uint8_t)tdata;
				DRVt2 = (uint8_t)(tdata >> 8);
				if(DRVwr)
				{
					DRVt1 = (uint8_t)DRVwr;
					DRVt2 = (uint8_t)(DRVwr >> 8);
				}			
			}
			DRVadr++;
			if(DRVadr > 0x04)
			{
				 DRVadr = 0;
				 DRVwr = 0;
			}				 
			spi_put(&DRVSPI, DRVt2);
			DRVstate = 1;
			break;
		case 1:
			if(spi_is_tx_ready(&DRVSPI))
			{
				DRVr2 = spi_get(&DRVSPI);
				spi_put(&DRVSPI, DRVt1);
				DRVstate = 2;		
			}
			break;
		case 2:
			if(spi_is_tx_ready(&DRVSPI))
			{
				DRVr1 = spi_get(&DRVSPI);
				DRVraw = ((uint16_t)DRVr2 << 8) | DRVr1;
				ioport_set_pin_level(DRV_CS, true);
				if(DRVraw == 0xffff) // DRV not enabled. MISO has pull up
				{
					DRVreg[0] = DRVreg[1] = DRVreg[2] = DRVreg[3] = 0;
					drv_disable();
				}
				if(!(DRVraw & bmDRVread))
				{
					tmpadr = (DRVraw & 0x7800) >> 11;
					if((tmpadr < 0x04) && !prevDRVwr)
					{
						DRVreg[tmpadr] = DRVraw & 0x07ff;					
					}
				}
				DRVstate = 0;
			}
			break;
	}
}

// ADC zero calibration on current measure channels
static void cal_sm(void)
{
	if(calstate == 0)	
	{
		cal_a1 = 0;
		cal_a2 = 0;
	}
	cal_a1 += ADCso1;
	cal_a2 += ADCso2;		
	if(calstate >= 15)
	{
		so1_zero = cal_a1 >> 4;
		so2_zero = cal_a2 >> 4;
		caldone = 1;
		calstate = 0;
	}
	calstate++;
}

static int16_t current2raw(int16_t current)
{
	return clip16(q8mul(current, I2RGAIN), -ILIMRAW, ILIMRAW);
}

static int16_t raw2current(int16_t raw)
{
	return q8mul(raw, R2IGAIN);
}

static void sysservice(void)
{
	Kp = 1;
	Ki = 0.5;
	//iMeasured_value = 0;
	iDifference = 0;
	iControl = 0;
	iError = 0;
	iIntegral = 0;
	int measure_time = 10;
	int16_t iMeasured_velocity_setpoint;
	iPrevious_value = iMeasured_value;
	
	if(ledcnt++ >= measure_time)
	{
		ledcnt = 0;
		
		ioport_toggle_pin_level(LED1);
	
		iMeasured_value = qei_raw;
		iDifference =  iPrevious_value - iMeasured_value;
		// Als het verschil precies bij het resetten van de encoder is, creeër goede verschil.
		if(iDifference < -60000)
		{
			iPrevious_value = 65536 - iPrevious_value;
			iDifference = iMeasured_value + iPrevious_value;
		}
				
//		iMeasured_velocity = ((iDifference / (measure_time / 1000))*60) / 24500; // In RPM
		iMeasured_velocity = iDifference / 4;
		//divided by 4 = (60 * 10)/(measure_time * 245);
		
//		iMeasured_velocity_setpoint = iMeasured_velocity;
		iError = setpoint - iMeasured_velocity;
		iIntegral = iError + iPreviousError;
		// Regeling
		iControl = iError * Kp + iIntegral * Ki;
		// Stuur door naar stroom regeling
		i_setpoint = raw2current((int16_t)iControl);
		iPreviousError = iError;
		
	}
	if(wdcnt++ >= WATCHDOG)
	{
		wdcnt = WATCHDOG;
		cenable = 0;
	}
	if(ADCntc < OVERTEMP)
	{
		if (ntccnt <= 100) ntccnt++;
	}	  
	else
	{
		ntccnt=0;  
		overtemp = 0;
	}	 
	if(ntccnt >= 100) 
	{
		cenable = 0;
		overtemp = 1;
	}		
		
    DRV8301_sm();
	if (cenable && !senable)
	{
        if((DRVadr == 0x00) && (DRVreg[3] != DRV3REG))
        {
			calstate = 0;
	        // Shunt amplifier 0=10V/V 1=20V/V 2=40V/V 3=80V/V
	        DRVwr = (uint16_t)bmDRVctrl2 | DRV3REG;
        }
		if(!caldone && (DRVreg[3] == DRV3REG))
		{
			cal_sm();
		}			
        if((DRVadr == 0x00) && !DRVwr && !(DRVreg[2] & 0x008) && caldone)
        {
	        // PWMmode = 1 (3 pins) OCADJ_SET = 3 (+-86mV) 10 = (+-197mV) 14 = (+-317mV)
	        DRVwr = (uint16_t)bmDRVctrl1 | 0x008 | (10 << 6);
        }
	    i_i = 0;
        if((DRVreg[2] & 0x008) && caldone)
        {
	        senable = 1;
			ioport_set_pin_level(LED0, true);
        }
	}			
	if((!cenable && senable) || (cenable && senable && (DRVreg[3] != DRV3REG)) )
	{
        if(DRVreg[2] & 0x008)
        {
	        // PWMmode = 0 (6 pins) OCADJ_SET = 3 (+-86mV) 10 = (+-197mV) 14 = (+-317mV)
	        DRVwr = (uint16_t)bmDRVctrl1 | 0x000 | (10 << 6);
        }
        if(!(DRVreg[2] & 0x008))
        {
			drv_disable();
        }
	}
}



static uint8_t getpacket(void)
{
	uint8_t i;
	int8_t crc;
	if(rx_counter_USARTF0 >= 8)
	{
		inpacket[0] = uart_getcharF0(0x00);
		if(inpacket[0] == SOF)
		{
			crc = 0;
			crc = crc8(crc, inpacket[0]);
			for(i = 1; i <= 6 ; i++)
			{
				inpacket[i] = uart_getcharF0(0x00);
				crc = crc8(crc, inpacket[i]);
			}
			inpacket[7] = uart_getcharF0(0x00);
			rcrc = crc;
			
			// overrule CRC if CRC data = 0x00
			if((inpacket[7] == rcrc) ||(inpacket[7] == 0x00) )
				return 1;
		}
	}
	return 0;
}

static void retranspacket(void)
{
	uint8_t i;
	uint8_t crc;
	packett *pp;
	
	pp = (packett *)&inpacket[0];
	pp->type--;
	crc = 0;
	for(i = 0; i <= 6 ; i++)
	{
		crc = crc8(crc, inpacket[i]);
		uart_putcharF0(inpacket[i]);
	}
	uart_putcharF0(crc);
}

static void cvpacket(void)
{
	uint8_t i;

	uint8_t crc;
	packett *pp;
	irqflags_t flags;
//	int16_t rawc;
	
	pp = (packett *)&outpacket[0];
	pp->sof = SOF;
	pp->type = 0x55;
	pp->cmd = (senable & 0x01) | (overtemp << 2) | (undervoltage << 3) | (inpacket[2] & 0xf0);
	if (cenable > 1)
	{
		flags = cpu_irq_save();
		pp->data1 = (uint16_t)ADCntc2;
		cpu_irq_restore(flags);
	}
	else
	{
		flags = cpu_irq_save();
		pp->data1 = (uint16_t)iMeasured_velocity;
		cpu_irq_restore(flags);
	}
	//int measured_val = qei_val / 10;		
	flags = cpu_irq_save();
	//rawc = (uint16_t)ifilt;
	cpu_irq_restore(flags);
	
	pp->data2 = setpoint;
	
	crc = 0;
	for(i = 0; i <= 6 ; i++)
	{
		crc = crc8(crc, outpacket[i]);
		uart_putcharF0(outpacket[i]);
	}
	uart_putcharF0(crc);
}

static void getsetpoints(void)
{
	packett *pp;
	
	// reset watchdog counter
	wdcnt = 0;
	
	pp = (packett *)&inpacket[0];
	if((pp->cmd & bmENABLE) && ((pp->cmd & bmMODEC) == bmMODEC) && !overtemp)
	{
		cenable = 1;
		if ((pp->cmd & bmMODEV) == bmMODEV) cenable++;
		//i_setpoint = current2raw((int16_t)pp->data1);
		setpoint = (int16_t)pp->data1;
		
	}
	else
	{
		cenable = 0;
	}
}

int main (void)
{
	const usart_serial_options_t usart_serial_options =
	{
		.baudrate     = 115200,
		.charlength   = USART_CHSIZE_8BIT_gc,
		.paritytype   = USART_PMODE_DISABLED_gc,
		.stopbits     = false
	};
	
	//uint8_t i;

	board_init();	
	delay_ms(4);
	
	ioport_set_pin_level(LED0, true);
	ioport_set_pin_level(DRV_CS, true);
	
	adc_init();
	qei_init();
	
	/* setup TimerCounter D0 for single-slope PWM 40KHz at 8*cpu clock = 256MHz */
	tc_enable(&TCD0);
	tc_set_overflow_interrupt_callback(&TCD0, &PWMtop);
	tc_set_wgm(&TCD0, TC_WG_SS);
	tc_write_period(&TCD0, PWMPER);
	tc_set_overflow_interrupt_level(&TCD0, TC_INT_LVL_MED);
	tc_hires_set_mode(&HIRESD, HIRES_HREN_TC0_gc | SUPERHIGHRES); // 
	tc_write_cc(&TCD0, TC_CCC, 0); // init at 0% PWM
	tc_write_cc(&TCD0, TC_CCD, 0); // init at 0% PWM
	conv_trig = HALFPWM;
	write_conv_trig(conv_trig);
	tc_enable_cc_channels(&TCD0,TC_CCCEN); // enable compare C to pin PD2 = PWM_A
	tc_enable_cc_channels(&TCD0,TC_CCDEN); // enable compare D to pin PD3 = PWM_B
	tc_write_clock_source(&TCD0, TC_CLKSEL_DIV1_gc);
	
	/* setup TimerCounter C0 for single-slope 1KHZ  */
	tc_enable(&TCC0);
	tc_set_overflow_interrupt_callback(&TCC0, &sysservice);
	tc_set_wgm(&TCC0, TC_WG_SS);
	tc_write_period(&TCC0, SERVICEPER);
	tc_set_overflow_interrupt_level(&TCC0, TC_INT_LVL_LO);
	tc_write_clock_source(&TCC0, TC_CLKSEL_DIV1_gc);

	/* setup SPI port for DRV, mode 1 and 1MHz */
	spi_master_init(&DRVSPI);
	spi_master_setup_device(&DRVSPI, NULL, SPI_MODE_1, 1000000, 0);
	spi_enable(&DRVSPI);

	// Initialize Serial Interface 1 using Stdio Library
	usart_serial_init(&SER1, &usart_serial_options);
	usart_set_rx_interrupt_level(&SER1, USART_INT_LVL_LO);
	usart_set_tx_interrupt_level(&SER1, USART_INT_LVL_LO);
	set_stdinout();

	cpu_irq_enable();
//	ioport_set_pin_level(DRV_EN , true);
	delay_ms(10);
	ioport_set_pin_level(LED0, false);
	ioport_set_pin_level(RS422RE, false); 
	ioport_set_pin_level(RS422DE, true);
	
	i_con = 0;
	i_i = 0;
	i_setpoint = 0;
	cenable = 0;
	
	while(1)
	{	
		if (ADCvpwr < SHUTDOWNVOLTAGE)
		{
			ioport_set_pin_level(DRV_EN , false);	
			drv_disable();
			undervoltage = 1;
		}
		if (ADCvpwr > STARTUPVOLTAGE)
		{
			ioport_set_pin_level(DRV_EN , true);
			undervoltage = 0;
		}
		if(getpacket())
		{
			switch(inpacket[1])
			{
				case 0xab:
				case 0xac:
				case 0xad:
				case 0xae:
				case 0xaf:
					retranspacket();
					break;
				case 0xaa:
					cvpacket();
					getsetpoints();
					break;
			}
		}
//			cenable = 1;
//			i_setpoint = 100;
//		printf("qei:%6d I:%4d so1:%4d so2:%4d vpwr:%4d ntc:%4d con:%d err:%d %d\r\n", qei_val, cv_I, ADCso1 , ADCso2, ADCvpwr, ADCntc, i_con, i_err, (int16_t)((UPWMGAIN << 4)/(ADCvpwr >> 3)) );	
//		delay_ms(20);
	}
}
