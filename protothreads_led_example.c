/**
 * Cornell Rocketry Team
 * Electrical and Software 
 *	File: CRTOS.c
 *  Created: 1/7/2020 3:20:12 PM
 *  Author: sdd58
 */ 


/**
 * Example use of prothreads structures, blinks LED on
 * the Sparkfun SAMD21 dev board. 
 */
#include "sam.h"
#include "pt_crt_0_0_1.h"

static struct pt pt_LED_1, pt_LED_2;

static PT_THREAD (protothread_LED1(struct pt *pt)){
	PT_BEGIN(pt);
	
	PORT->Group[0].DIRSET.reg |= PORT_PA27;
	PORT->Group[0].OUTSET.reg |= PORT_PA27;

	while(1) {
		PT_YIELD_TIME_msec(10000);
		PORT->Group[0].OUTTGL.reg |= PORT_PA27;
	}

	PT_END(pt);
}

static PT_THREAD (protothread_LED2(struct pt *pt)){
	PT_BEGIN(pt);
	
	PORT->Group[1].DIRSET.reg |= PORT_PB03;
	PORT->Group[1].OUTSET.reg |= PORT_PB03;
	
	while(1){
		PT_YIELD_TIME_msec(2500);
		PORT->Group[1].OUTTGL.reg |= PORT_PB03;
	}

	PT_END(pt);
}

int main3(void)
{
    /* Initialize the SAM system */
    SystemInit(); CLK_setup(); 
	

	__enable_irq();

	/* Initialize the CRT configuration */
	PT_setup();

	PT_INIT(&pt_LED_1);
	PT_INIT(&pt_LED_2);

	while(1){
		PT_SCHEDULE(protothread_LED1(&pt_LED_1));
		PT_SCHEDULE(protothread_LED2(&pt_LED_2));
	}

    /* Replace with your application code */
	
}

