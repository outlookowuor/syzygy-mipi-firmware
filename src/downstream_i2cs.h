#ifndef DOWNSTREAM_H
#define DOWNSTREAM_H

int setup_downstream_i2cs();

void bridge_i2c_receive(uint8_t data);
void bridge_i2c_request(uint8_t *buffer);
void bridge_i2c_stop(uint8_t length);
void bridge_i2c_restart_request(uint8_t *buffer);
#endif 