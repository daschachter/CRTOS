/*
 * crt_usart.c
 *
 * Created: 1/8/2020 2:21:40 PM
 *  Author: dts18
 */

/******************
* Good resources:
*  - Getting Started with the SAM D21 Xplained Pro without ASF: https://www.digikey.com/eewiki/display/microcontroller/Getting+Started+with+the+SAM+D21+Xplained+Pro+without+ASF
*  - SAM D21 Clock System Configuration: https://microchipdeveloper.com/32arm:samd21-clock-system-configuration
*  - https://www.avrfreaks.net/forum/no-asf-samd21-sercom3-tutorial-help
******************/


#include "crt_usart.h"

#define OSC8M_FREQ 8000000


/**********************
RX_PINMUX: Pinmux for pin/sercom (e.g. PINMUX_PA11C_SERCOM0_PAD3)
TX_PINMUX: Pinmux for pin/sercom (e.g. PINMUX_PA10C_SERCOM0_PAD2) 
Pinmux definitions are found in samd21g18a.h
***********************/
void usart_pin_init(uint32_t RX_PINMUX, uint32_t TX_PINMUX) {
	// Extract port and pin information
	uint8_t rx_port = (uint8_t)((RX_PINMUX >> 16) / 32);
	uint8_t tx_port = (uint8_t)((TX_PINMUX >> 16) / 32);
	uint32_t rx_pin = (RX_PINMUX >> 16) - (rx_port * 32);
	uint32_t tx_pin = (TX_PINMUX >> 16) - (tx_port * 32);
	
	// Configure TX
	PORT->Group[tx_port].DIRSET.reg = (1 << tx_pin);
	PORT->Group[tx_port].PINCFG[tx_pin].reg |= PORT_PINCFG_INEN;
	PORT->Group[tx_port].PINCFG[tx_pin].reg |= PORT_PINCFG_PMUXEN;
	PORT->Group[tx_port].PMUX[tx_pin >> 1].reg &= ~(0xF << (4 * ((TX_PINMUX >> 16) & 0x01u)));
	PORT->Group[tx_port].PMUX[tx_pin >> 1].reg |= (uint8_t)((TX_PINMUX & 0x0000FFFF) << (4 * ((TX_PINMUX >> 16) & 0x01u)));
	// More intuitive approach for PMUX:
	// PORT->Group[tx_port].PMUX[tx_pin >> 1].bit.PMUXE = PORT_PMUX_PMUXE_C_Val;
	
	// Configure RX
	PORT->Group[rx_port].DIRCLR.reg = (1 << rx_pin);
	PORT->Group[rx_port].PINCFG[rx_pin].reg |= PORT_PINCFG_INEN;
	PORT->Group[rx_port].PINCFG[rx_pin].reg &= ~PORT_PINCFG_PULLEN;
	PORT->Group[rx_port].PINCFG[rx_pin].reg |= PORT_PINCFG_PMUXEN;
	PORT->Group[rx_port].PMUX[rx_pin >> 1].reg &= ~(0xF << (4 * ((RX_PINMUX >> 16) & 0x01u)));
	PORT->Group[rx_port].PMUX[rx_pin >> 1].reg |= (uint8_t)((RX_PINMUX & 0x0000FFFF) << (4 * ((RX_PINMUX >> 16) & 0x01u)));
	// More intuitive approach for PMUX:
	// PORT->Group[rx_port].PMUX[rx_pin >> 1].bit.PMUXO = PORT_PMUX_PMUXE_C_Val; // NOTE: ...PMUXE... and ...PMUXO... are the same
}


/**********************
* channel: Sercom being used for USART (e.g. SERCOM3)
* baud: desired baud rate (e.g. 9600)
* initializes USART for 16x oversampling
***********************/
void usart_init(Sercom* channel, uint32_t baud, uint8_t RX_PAD, uint8_t TX_PAD) {

	
	/* By setting the DORD bit LSB is transmitted first and setting the RXPO bit as x corresponding SERCOM PAD[x] will be used 
	for data reception, PAD[y] will be used as TxD pin by setting TXPO bit as 0,16x over-sampling is selected by setting the 
	SAMPR bit as 0,Generic clock is enabled in all sleep modes by setting RUNSTDBY bit as 1, USART clock mode is selected as 
	USART with internal clock by setting MODE bit into 1.*/
	channel->USART.CTRLA.reg =   SERCOM_USART_CTRLA_DORD          | 
								SERCOM_USART_CTRLA_RXPO(RX_PAD)  |
								SERCOM_USART_CTRLA_TXPO(TX_PAD)  |
								SERCOM_USART_CTRLA_SAMPR(0x0)    |
								SERCOM_USART_CTRLA_RUNSTDBY      |
								SERCOM_USART_CTRLA_MODE_USART_INT_CLK;
								
	/* baud register value corresponds to the device communication baud rate
	see documentation for arithmetic baud rate formula */
	channel->USART.BAUD.reg = calculate_baud_value(baud, OSC8M_FREQ, 16);
	
	/* 8-bits size is selected as character size by setting the bit CHSIZE as 0,
	TXEN bit and RXEN bits are set to enable the transmitter and receiver. */
	channel->USART.CTRLB.reg =  SERCOM_USART_CTRLB_CHSIZE(0x0) |
								SERCOM_USART_CTRLB_TXEN |
								SERCOM_USART_CTRLB_RXEN;
	
	/* synchronization busy */
	while(channel->USART.SYNCBUSY.bit.CTRLB);
	/* receive complete interrupt set */
	channel->USART.INTENSET.reg = SERCOM_USART_INTFLAG_RXC;
	/* sercom peripheral enabled */
	channel->USART.CTRLA.reg |= SERCOM_USART_CTRLA_ENABLE;
	/* synchronization busy */
	while(channel->USART.SYNCBUSY.reg & SERCOM_USART_SYNCBUSY_ENABLE);
}

/**********************
Returns 0 if not busy, 1 otherwise
***********************/
int usart_is_busy(Sercom * channel) {
	return channel->USART.INTFLAG.bit.DRE == 0;
}

/**********************
Sends one byte of data
***********************/
void usart_transmit_byte(Sercom * channel, uint8_t send_byte) {
	channel->USART.DATA.reg = send_byte;
}

/**********************
Returns 0 if not available, 1 otherwise
***********************/
int usart_data_available(Sercom * channel) {
	return  channel->USART.INTFLAG.bit.RXC;
}

/**********************
Receive one byte of data
***********************/
int usart_receive_byte(Sercom * channel) {
	return channel->USART.DATA.reg;
}


