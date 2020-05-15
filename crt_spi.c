/*
 * crt_spi.c
 *
 * Created: 1/12/2020 12:50:17 PM
 *  Author: dts18
 */ 

/******************
* Good resources:
*  - http://ww1.microchip.com/downloads/en/AppNotes/00002465A.pdf
******************/

/**********************
MOSI_PINMUX: Pinmux for pin/sercom (e.g. PINMUX_PA11C_SERCOM0_PAD3)
MISO_PINMUX: Pinmux for pin/sercom (e.g. PINMUX_PA10C_SERCOM0_PAD2)
SCK_PINMUX: Pinmux for pin/sercom (e.g. PINMUX_PA11C_SERCOM0_PAD3)
SS_PINMUX: Pinmux for pin/sercom (e.g. PINMUX_PA10C_SERCOM0_PAD2)
Pinmux definitions are found in samd21g18a.h
***********************/

#include "crt_sercom.h"


#define OSC8M_FREQ 8000000
#define MAX_CHARS 64

void spi_pin_init(uint32_t MOSI_PINMUX, uint32_t MISO_PINMUX, uint32_t SCK_PINMUX, uint32_t SS_PINMUX) {
	// Extract port and pin information
	uint8_t mosi_port = (uint8_t)((MOSI_PINMUX >> 16) / 32);
	uint8_t miso_port = (uint8_t)((MISO_PINMUX >> 16) / 32);
	uint8_t sck_port = (uint8_t)((SCK_PINMUX >> 16) / 32);
	uint8_t ss_port = (uint8_t)((SS_PINMUX >> 16) / 32);
	
	uint32_t mosi_pin = (MOSI_PINMUX >> 16) - (mosi_port * 32);
	uint32_t miso_pin = (MISO_PINMUX >> 16) - (miso_port * 32);
	uint32_t sck_pin = (SCK_PINMUX >> 16) - (sck_port * 32);
	uint32_t ss_pin = (SS_PINMUX >> 16) - (ss_port * 32);
	
	// Configure MOSI
	PORT->Group[mosi_port].DIRSET.reg = (1 << mosi_pin);
	PORT->Group[mosi_port].PINCFG[mosi_pin].reg |= PORT_PINCFG_INEN;
	PORT->Group[mosi_port].PINCFG[mosi_pin].reg |= PORT_PINCFG_PMUXEN;
	PORT->Group[mosi_port].PMUX[mosi_pin >> 1].reg &= ~(0xF << (4 * ((MOSI_PINMUX >> 16) & 0x01u)));
	PORT->Group[mosi_port].PMUX[mosi_pin >> 1].reg |= (uint8_t)((MOSI_PINMUX & 0x0000FFFF) << (4 * ((MOSI_PINMUX >> 16) & 0x01u)));
	
	// Configure MISO
	PORT->Group[miso_port].DIRCLR.reg = (1 << miso_pin);
	PORT->Group[miso_port].PINCFG[miso_pin].reg |= PORT_PINCFG_INEN;
	PORT->Group[miso_port].PINCFG[miso_pin].reg |= PORT_PINCFG_PMUXEN;
	PORT->Group[miso_port].PINCFG[miso_pin].reg &= ~PORT_PINCFG_PULLEN;
	PORT->Group[miso_port].PMUX[miso_pin >> 1].reg &= ~(0xF << (4 * ((MISO_PINMUX >> 16) & 0x01u)));
	PORT->Group[miso_port].PMUX[miso_pin >> 1].reg |= (uint8_t)((MISO_PINMUX & 0x0000FFFF) << (4 * ((MISO_PINMUX >> 16) & 0x01u)));
	
	// Configure SCK
	PORT->Group[sck_port].DIRSET.reg = (1 << sck_pin);
	PORT->Group[sck_port].PINCFG[sck_pin].reg |= PORT_PINCFG_INEN;
	PORT->Group[sck_port].PINCFG[sck_pin].reg |= PORT_PINCFG_PMUXEN;
	PORT->Group[sck_port].PMUX[sck_pin >> 1].reg &= ~(0xF << (4 * ((SCK_PINMUX >> 16) & 0x01u)));
	PORT->Group[sck_port].PMUX[sck_pin >> 1].reg |= (uint8_t)((SCK_PINMUX & 0x0000FFFF) << (4 * ((SCK_PINMUX >> 16) & 0x01u)));
	
	// Configure SS
	PORT->Group[ss_port].DIRSET.reg = (1 << ss_pin);
	PORT->Group[ss_port].PINCFG[ss_pin].reg |= PORT_PINCFG_INEN;
	PORT->Group[ss_port].PINCFG[ss_pin].reg |= PORT_PINCFG_PMUXEN;
	PORT->Group[ss_port].PMUX[ss_pin >> 1].reg &= ~(0xF << (4 * ((SS_PINMUX >> 16) & 0x01u)));
	PORT->Group[ss_port].PMUX[ss_pin >> 1].reg |= (uint8_t)((SS_PINMUX & 0x0000FFFF) << (4 * ((SS_PINMUX >> 16) & 0x01u)));
	PORT->Group[ss_port].OUTSET.reg |= (1 << ss_pin);
}

/**********************
* channel: Sercom being used for SPI (e.g. SERCOM3)
* baud: desired baud rate (e.g. 9600)
* DOPO: value for the DOPO register
         -----------------------------------------
			 DOPO |   DO   |   SCK   | Slave_SS
			 0x0  | PAD[0] |  PAD[1] |  PAD[2]
			 0x1  | PAD[2] |  PAD[3] |  PAD[1]
			 0x2  | PAD[3] |  PAD[1] |  PAD[2]
			 0x3  | PAD[0] |  PAD[3] |  PAD[1]
         -----------------------------------------
* MISO_PAD: pad that will be used 
* initializes SPI for 16x oversampling
* CPOL=0 and CPHA=0
* 8-bit frames used (no addressing)
* LSB transmitted first
***********************/
void spi_init_master(Sercom* channel, uint32_t baud, uint8_t DOPO, uint8_t MISO_PAD) {	
	channel->SPI.CTRLA.reg =	SERCOM_SPI_CTRLA_MODE(0x3)		|
								SERCOM_SPI_CTRLA_FORM(0x0)		|
								SERCOM_SPI_CTRLA_DOPO(DOPO)		| 
								SERCOM_SPI_CTRLA_DIPO(MISO_PAD)	|
								SERCOM_SPI_CTRLA_DORD;
	
								
	channel->SPI.BAUD.reg = calculate_baud_value(baud, OSC8M_FREQ, 16);
	
	channel->SPI.CTRLA.reg &= ~SERCOM_SPI_CTRLA_CPOL; // CPOL = 0
	channel->SPI.CTRLA.reg &= ~SERCOM_SPI_CTRLA_CPHA; // CPHA = 0
	
	channel->SPI.CTRLB.reg =	SERCOM_SPI_CTRLB_CHSIZE(0x0) |
								SERCOM_SPI_CTRLB_MSSEN |
								SERCOM_SPI_CTRLB_RXEN;
								
	/* synchronization busy */
	while(channel->SPI.SYNCBUSY.bit.CTRLB);
	/* sercom peripheral enabled */
	channel->SPI.CTRLA.reg |= SERCOM_SPI_CTRLA_ENABLE;
	while(channel->SPI.SYNCBUSY.bit.ENABLE);
}


void spi_init_slave(Sercom* channel, uint32_t baud) {
	
}

char spi_tx_buffer[MAX_CHARS] = "hello";
char spi_rx_buffer[MAX_CHARS];
int spi_tx_index = 0;
int spi_rx_index = 0;
uint8_t volatile spi_tx_done = 0;
void spi_master_send(Sercom* sercom, uint32_t SS_PINMUX) {
	uint8_t ss_port = (uint8_t)((SS_PINMUX >> 16) / 32);
	uint32_t ss_pin = (SS_PINMUX >> 16) - (ss_port * 32);
	
	// Configure variables
	spi_tx_index = 0;
	spi_rx_index = 0;
	spi_tx_done = 0;
	
	// Set SS to low (begin communication)
	PORT->Group[ss_port].OUTCLR.reg |= (1 << ss_pin);
	
	// Enable interrupts
	sercom->SPI.INTENSET.reg = SERCOM_SPI_INTENSET_DRE | SERCOM_SPI_INTENSET_RXC;
	
	// Wait for transmission to complete
	while(!spi_tx_done);
	
	// Set SS to high (end communication)
	PORT->Group[ss_port].OUTSET.reg |= (1 << ss_pin);	
}

#ifdef SERCOM0_SPI
void SERCOM0_Handler() {
	/* Data register empty flag set */
	if(SERCOM0->SPI.INTFLAG.bit.DRE && SERCOM0->SPI.INTENSET.bit.DRE) {
		SERCOM0->SPI.DATA.reg = spi_tx_buffer[spi_tx_index++];
		if(spi_tx_index >= MAX_CHARS || spi_tx_buffer[spi_tx_index] == 0) {
			SERCOM0->SPI.INTENCLR.reg = SERCOM_SPI_INTENCLR_DRE;
		}
	}
	
	/* Receive complete interrupt */
	if(SERCOM0->SPI.INTFLAG.bit.RXC && SERCOM0->SPI.INTENSET.bit.RXC) {
		spi_rx_buffer[spi_rx_index++] = SERCOM0->SPI.DATA.reg;
		if(spi_rx_index >= MAX_CHARS || spi_tx_buffer[spi_tx_index] == 0)  { // if transmission is done
			SERCOM0->SPI.INTENCLR.reg = SERCOM_SPI_INTENCLR_RXC;
			spi_tx_done = 1;
		}
	}
}
#endif

#ifdef SERCOM1_SPI

#endif

// int main(void) {
// 	SystemInit(); CLK_setup();
// 	sercom_init(SERCOM0, 0);
// 	spi_pin_init(PINMUX_PA08C_SERCOM0_PAD0, PINMUX_PA11C_SERCOM0_PAD3, PINMUX_PA08C_SERCOM0_PAD0, PINMUX_PA10C_SERCOM0_PAD2);
// 	spi_init_master(SERCOM0, 9600, 0x0, 0x3);
// 	
// 	PORT->Group[0].DIRSET.reg |= PORT_PA27;
// 	PORT->Group[0].OUTSET.reg |= PORT_PA27;
// 	
// 	while(1) {
// 		spi_master_send(SERCOM0, PINMUX_PA10C_SERCOM0_PAD2);
// 		PORT->Group[0].OUTCLR.reg |= PORT_PA27;
// // 		for(int i = 0; i < 200000; i++) {
// // 			for(int i = 0; i < 200000; i++) {
// // 				
// // 			}
// // 		}
// 	}
// 	
// }