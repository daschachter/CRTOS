/*
 * crt_spi.h
 *
 * Created: 1/12/2020 12:50:07 PM
 *  Author: dts18
 */ 


#ifndef CRT_SPI_H_
#define CRT_SPI_H_

void spi_pin_init(uint32_t MOSI_PINMUX, uint32_t MISO_PINMUX, uint32_t SCK_PINMUX, uint32_t SS_PINMUX);
void spi_init_master(Sercom* channel, uint32_t baud, uint8_t DOPO, uint8_t MISO_PAD);
void spi_init_slave(Sercom* channel, uint32_t baud);
void spi_master_send(Sercom* sercom, uint32_t SS_PINMUX);




#endif /* CRT_SPI_H_ */