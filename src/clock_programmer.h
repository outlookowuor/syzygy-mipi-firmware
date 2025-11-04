#ifndef CLOCK_PROGRAMMER_H
#define CLOCK_PROGRAMMER_H

#define CLOCK_PROGRAMMER_I2C_ADDRESS 0x17


void clock_programmer_i2c_receive(uint8_t data);
void clock_programmer_i2c_request(uint8_t *buffer);
void clock_programmer_i2c_stop(uint8_t length);
void clock_programmer_i2c_restart_request(uint8_t *buffer);

#endif 