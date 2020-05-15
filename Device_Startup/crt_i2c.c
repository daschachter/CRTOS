/*
 * crt_i2c.c
 *
 * Created: 2/20/2020 5:16:02 PM
 *  Author: dts18
 */ 

void i2c_pin_init(uint32_t SDA_PINMUX, uint32_t SCK_PINMUX) {
	uint8_t sda_port = (uint8_t)((SDA_PINMUX >> 16) / 32);
	uint8_t sck_port = (uint8_t)((SCK_PINMUX >> 16) / 32);
}