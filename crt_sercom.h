/*
 * crt_sercom.h
 *
 * Created: 1/12/2020 2:50:54 PM
 *  Author: dts86,sdd58
 */ 

#include "sam.h"


#ifndef CRT_SERCOM_H_
#define CRT_SERCOM_H_

void sercom_init( Sercom * channel, uint8_t num );

uint16_t calculate_baud_value(const uint32_t baudrate, 
							  const uint32_t peripheral_clock, 
							  uint8_t sample_num);


#endif /* CRT_SERCOM_H_ */