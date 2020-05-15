/**
 * Cornell Rocketry Team
 * Electrical and Software 
 *	File: crt_usart.c
 *  Created: 1/7/2020 3:20:12 PM
 *  Author: dts86, sdd58
 */ 



#ifndef CRT_USART_H_
#define CRT_USART_H_

#include "sam.h"
#include "crt_sercom.h"

#define PORTGROUP_A 0
#define PORTGROUP_B 1

#define USART_BAUD_RATE 9600
#define USART_SAMPLE_NUM 16

void usart_pin_init( uint32_t RX_PINMUX, uint32_t TX_PINMUX);
void usart_init(Sercom* channel, uint32_t baud, uint8_t RX_PAD, uint8_t TX_PAD);

int usart_is_busy(Sercom * channel);
void usart_transmit_byte(Sercom * channel, uint8_t send_byte);
int usart_data_available(Sercom * channel);
int usart_receive_byte(Sercom * channel);  

#endif CRT_USART_H_