/**
 * Cornell Rocketry Team
 * Electrical and Software Subteam
 *	File: crt_config_0_0_1.h
 *  Created: 1/7/2020 7:54:37 PM
 *  Author: sdd58
 */ 


#ifndef CRT_CONFIG_0_0_1_H_
#define CRT_CONFIG_0_0_1_H_

#define NULL 0

// I/O Ports definitions
#define PORTA     (0ul)
#define PORTB     (1ul)

// GCLK_MAIN Clock output IO Pin Definition
#define GCLK_MAIN_OUTPUT_PORT       PORTA
#define GCLK_MAIN_OUTPUT_PIN_NUMBER (14ul)
#define GCLK_MAIN_OUTPUT_PIN_MASK   PORT_PA14

// Constants for Clock Generators
#define GENERIC_CLOCK_GENERATOR_MAIN      (0u)
#define GENERIC_CLOCK_GENERATOR_XOSC32K   (1u)
#define GENERIC_CLOCK_GENERATOR_OSCULP32K (2u) /* Initialized at reset for WDT */
#define GENERIC_CLOCK_GENERATOR_OSC8M     (3u)
// Constants for Clock multiplexers
#define GENERIC_CLOCK_MULTIPLEXER_DFLL48M (0u)

// Constants for DFLL48M
#define MAIN_CLK_FREQ (48000000u)
#define EXT_32K_CLK_FREQ (32768u)



#endif /* CRT_CONFIG_0_0_1_H_ */