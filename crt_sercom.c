/*
 * crt_sercom.c
 *
 * Created: 1/12/2020 2:19:37 PM
 *  Author: dts86,sdd58
 */ 

#include "crt_sercom.h"


/**********************
* https://microchipdeveloper.com/32arm:samd21-clock-system-configuration
* Uses OSC8M (i.e 8MHz internal oscillator)
**********************/
void sercom_init( Sercom * channel, uint8_t num ) {
	
	SYSCTRL->OSC8M.bit.PRESC = 0; // no prescalar (is 8 on reset)
	SYSCTRL->OSC8M.reg |= 1 << SYSCTRL_OSC8M_ENABLE_Pos; // enable source
	
	GCLK->GENDIV.bit.ID = 0x03; // select GCLK_GEN[3]
	GCLK->GENDIV.bit.DIV = 0; // no prescalar
	
	GCLK->GENCTRL.bit.ID = 0x03; // select GCLK_GEN[3]
	GCLK->GENCTRL.reg |= GCLK_GENCTRL_SRC_OSC8M; // OSC8M source
	GCLK->GENCTRL.bit.GENEN = 1; // enable generator
	
	// sercom peripheral channel
	switch(num) {
		case 0:
		GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_SERCOM0_CORE;
		break;
		case 1:
		GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_SERCOM1_CORE;
		break;
		case 2:
		GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_SERCOM2_CORE;
		break;
		case 3:
		GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_SERCOM3_CORE;
		break;
		case 4:
		GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_SERCOM4_CORE;
		break;
		case 5:
		GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_SERCOM5_CORE;
		break;
		default:
		GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_SERCOM0_CORE;
		break;
	}
	
	GCLK->CLKCTRL.reg |= GCLK_CLKCTRL_GEN_GCLK3; // select source GCLK_GEN[3]
	GCLK->CLKCTRL.bit.CLKEN = 1; // enable generic clock
	
	PM->APBCSEL.bit.APBCDIV = 0; // no prescalar
	switch(num) {
		case 0:
		PM->APBCMASK.bit.SERCOM0_ = 1;
		break;
		case 1:
		PM->APBCMASK.bit.SERCOM1_ = 1;
		break;
		case 2:
		PM->APBCMASK.bit.SERCOM2_ = 1;
		break;
		case 3:
		PM->APBCMASK.bit.SERCOM3_ = 1;
		break;
		case 4:
		PM->APBCMASK.bit.SERCOM4_ = 1;
		break;
		case 5:
		PM->APBCMASK.bit.SERCOM5_ = 1;
		break;
		default:
		PM->APBCMASK.bit.SERCOM0_ = 1;
		break;
	}
}

/**********************
* Calculate 64 bit division, ref can be found in
* http://en.wikipedia.org/wiki/Division_algorithm#Long_division
***********************/
static uint64_t long_division(uint64_t n, uint64_t d) {
	int32_t i;
	uint64_t q = 0, r = 0, bit_shift;
	for(i = 63; i >= 0; i--) {
		bit_shift = (uint64_t)1 << i;
		
		r = r << 1;
		
		if(n & bit_shift) {
			r |= 0x01;
		}
		
		if(r >= d) {
			r = r - d;
			q |= bit_shift;
		}
	}
	
	return q;
}

/**********************
* Calculates BAUD register value given in SAMD21 datasheet
***********************/
uint16_t calculate_baud_value(
const uint32_t baudrate,
const uint32_t peripheral_clock,
uint8_t sample_num)
{
	/* Temporary variables */
	uint64_t ratio = 0;
	uint64_t scale = 0;
	uint64_t baud_calculated = 0;
	uint64_t temp1;
	
	
	/* Calculate the BAUD value */
	temp1 = ((sample_num * (uint64_t) baudrate) << 32);
	ratio = long_division(temp1, peripheral_clock);
	scale = ((uint64_t)1 << 32) - ratio;
	baud_calculated = (65536 * scale) >> 32;
	
	return baud_calculated;
}
