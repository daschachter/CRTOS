/**
 * Cornell Rocketry Team
 * Electrical and Software 
 *	File: main.c
 *  Created: 1/7/2020 3:20:12 PM
 *  Author: dts86, sdd58
 */ 

#include "sam.h"
#include "pt_crt_0_0_1.h"

#include "crt_usart.h"
#include "crt_sercom.h"
#include "crt_adc.h"

// Using system calls (like sprintf)
// In properties, go to "Toolchain>General>Additional Specs" and select
// "Use syscall stubs"
// Maybe: https://startingelectronics.org/articles/atmel-AVR-8-bit/print-float-atmel-studio-7/
// http://ww1.microchip.com/downloads/en/DeviceDoc/Frequently-Asked-Questions-4.9.3.26.txt


static struct pt pt_getserbuff;
static struct pt pt_putserbuff;


void write_int_to_transmit(int16_t var) {
	int counter = 4;
	PT_send_buffer[counter + 1] = '\r';
	PT_send_buffer[counter + 2] = '\n';
	while(counter >= 0) {
		PT_send_buffer[counter--] = (var % 10) + 48;

		var = var / 10;
	}
}

void ADC_Handler(void) {
	if(ADC->INTFLAG.bit.RESRDY) {
		PORT->Group[0].OUTTGL.reg |= PORT_PA17;
		int16_t result = ADC->RESULT.reg;
		while(ADC->STATUS.bit.SYNCBUSY); // wait for synchronization
		
		write_int_to_transmit(result);
	}
}

int main(void) {
	SystemInit(); CLK_setup();
	
	sercom_init(SERCOM0, 0);
	usart_pin_init(PINMUX_PA11C_SERCOM0_PAD3, PINMUX_PA10C_SERCOM0_PAD2);
	usart_init(SERCOM0, 9600, 0x3, 0x1);
	
	// Configure LED
	PORT->Group[0].DIRSET.reg |= PORT_PA17;
	PORT->Group[0].OUTSET.reg |= PORT_PA17;
	
	//NVIC_EnableIRQ(SERCOM0_IRQn);
	
	PT_setup();
	PT_INIT(&pt_getserbuff);
	PT_INIT(&pt_putserbuff);
	
	//NVIC_EnableIRQ(ADC_IRQn);
	adc_init();
	
	while(1) {
		if(!PT_SCHEDULE(PT_get_serial_buffer(&pt_getserbuff))) {
			//ADC->SWTRIG.bit.START = 1;
			//while(ADC->STATUS.bit.SYNCBUSY); // wait for synchronization
			
			
			while(ADC->INTFLAG.bit.RESRDY == 0);
			PORT->Group[0].OUTTGL.reg |= PORT_PA17;
			int16_t result = ADC->RESULT.reg;
			while(ADC->STATUS.bit.SYNCBUSY); // wait for synchronization
					
			write_int_to_transmit(result);
			
			while(PT_put_serial_buffer(&pt_putserbuff) == PT_YIELDED);
		}
	}
}