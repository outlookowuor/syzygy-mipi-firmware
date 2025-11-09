#ifndef I2C_MUXER_H
#define I2C_MUXER_H

#define I2C_MUXER_I2C_ADDRESS 0x70
int setup_i2c_muxer();

bool i2c_muxer_i2c_receive(uint8_t data);
bool i2c_muxer_i2c_request(uint8_t *buffer);
void i2c_muxer_i2c_stop(uint8_t length);
bool i2c_muxer_i2c_restart_request(uint8_t *buffer);

#endif // I2C_MUXER_H