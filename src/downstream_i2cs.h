#ifndef DOWNSTREAM_H
#define DOWNSTREAM_H

#define MUXED_MIPI_I2C_ADDRESS 0x72


int setup_downstream_i2cs();

bool bridge_i2c_receive(uint8_t data);
bool bridge_i2c_request(uint8_t *buffer);
void bridge_i2c_stop(uint8_t length);
bool bridge_i2c_restart_request(uint8_t *buffer);
#endif 