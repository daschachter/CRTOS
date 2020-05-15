/*
 * crt_adc.c
 *
 * Created: 1/23/2020 7:45:37 PM
 *  Author: dts86
 */ 

// Select GCLK_ADC in SYSCTRL
// Enable ADC by writing 1 to enable bit in CTRLA
// Set trigger to be in free-running mode
// Set resolution (actually 12-bits already by default)
// Set up single-ended or diff mode
// Consider using averaging
// Consider using high resolution (16-bit mode with oversampling and decimation)
// DONT FORGET ABOUT SYNCHRONIZATION
// Read out the conversion in RESULT
// RESRDY flag when data is a available (set up interrupt using INTENSET?)

#include "crt_adc.h"

void adc_init() {
	/***************Clock Init***************/
	SYSCTRL->OSC8M.bit.PRESC = 0; // no prescalar (is 8 on reset)
	SYSCTRL->OSC8M.reg |= 1 << SYSCTRL_OSC8M_ENABLE_Pos; // enable source
	
	GCLK->GENDIV.bit.ID = 0x0; // select GCLK_GEN[0]
	GCLK->GENDIV.bit.DIV = 0; // no prescalar
	
	GCLK->GENCTRL.bit.ID = 0x0; // select GCLK_GEN[0]
	GCLK->GENCTRL.reg |= 1 << GCLK_GENCTRL_SRC_OSC8M; // OSC8M source
	GCLK->GENCTRL.bit.GENEN = 1; // enable generator
	
	GCLK->CLKCTRL.bit.ID = GCLK_CLKCTRL_ID_ADC; // select GCLK_ADC
	GCLK->CLKCTRL.reg |= GCLK_CLKCTRL_GEN_GCLK0; // select source GCLK[0] for the ADC
	GCLK->CLKCTRL.bit.CLKEN = 1; // enable generic clock
	
	/***************Setup Analog Input 1***************/
	PORT->Group[1].DIRCLR.reg = PORT_PB08; // Using PB08
	PORT->Group[1].PINCFG[8].reg |= PORT_PINCFG_INEN;
	PORT->Group[1].PINCFG[8].reg |= PORT_PINCFG_PMUXEN;
	PORT->Group[1].PMUX[8 >> 1].reg |= PORT_PMUX_PMUXE_B;
	
	/***************ADC Init***************/
	
	// Calibration
	uint64_t adc_calib = *(uint64_t*)NVMCTRL_CAL;
	uint16_t adc_linearity = (adc_calib >> 27) & 0xFF;
	uint16_t adc_biascal = (adc_calib >> 35) & 0x07;
	ADC->CALIB.bit.LINEARITY_CAL = adc_linearity;
	ADC->CALIB.bit.BIAS_CAL = adc_biascal;
	
	ADC->INPUTCTRL.reg |= (ADC_INPUTCTRL_MUXNEG_GND | ADC_INPUTCTRL_MUXPOS_PIN2); // set analog 1 as input
	while(ADC->STATUS.bit.SYNCBUSY); // wait for synchronization
	
	ADC->REFCTRL.reg |= ADC_REFCTRL_REFSEL_INT1V;
	
	ADC->CTRLB.reg &= ~ADC_CTRLB_DIFFMODE; // enable free-running, single-ended mode
	while(ADC->STATUS.bit.SYNCBUSY); // wait for synchronization
	
	ADC->CTRLB.reg |= ADC_CTRLB_FREERUN;
	while(ADC->STATUS.bit.SYNCBUSY); // wait for synchronization
	
	ADC->CTRLA.bit.ENABLE = 1; // enable ADC
	while(ADC->STATUS.bit.SYNCBUSY); // wait for synchronization
	
	ADC->INTENSET.reg |= ADC_INTENSET_RESRDY; // enable interrupt for result being ready
	while(ADC->STATUS.bit.SYNCBUSY); // wait for synchronization
	
	ADC->SAMPCTRL.reg |= 0x3F; // set sample length
	while(ADC->STATUS.bit.SYNCBUSY); // wait for synchronization

	ADC->INTFLAG.bit.RESRDY = 0x1; // Clear ready flag
	while(ADC->STATUS.bit.SYNCBUSY); // wait for synchronization
}