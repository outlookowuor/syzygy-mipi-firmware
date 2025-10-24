#ifndef DOWNSTREAM_H
#define DOWNSTREAM_H

#define MUXED_MIPI_I2C_ADDRESS 0x72


int setup_downstream_i2cs();

void muxed_mipi_i2c_write_byte(uint8_t data);
void muxed_mipi_i2c_read_byte(uint8_t *buffer);
void muxed_mipi_i2c_stop(uint8_t length);

#endif 