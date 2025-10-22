#ifndef I2C_MUXER_H
#define I2C_MUXER_H

#define I2C_MUXER_I2C_ADDRESS 0x70

int setup_i2c_muxer();

void i2c_muxer_i2c_write_byte(uint8_t data);
void i2c_muxer_i2c_read_byte(uint8_t data);
void i2c_muxer_i2c_stop(uint8_t length);

#endif // I2C_MUXER_H