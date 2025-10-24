#ifndef CLOCK_PROGRAMMER_H
#define CLOCK_PROGRAMMER_H

#define CLOCK_PROGRAMMER_I2C_ADDRESS 0x10


void clock_programmer_i2c_write_byte(uint8_t data);
void clock_programmer_i2c_read_byte(uint8_t *buffer);
void clock_programmer_i2c_stop(uint8_t length);

#endif 