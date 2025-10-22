#ifndef GPIO_EXPANDER_H
#define GPIO_EXPANDER_H

#define GPIO_EXPANDER_I2C_ADDRESS 0x50


int setup_gpio_expander();


void gpio_expander_i2c_write_byte(uint8_t data);
void gpio_expander_i2c_read_byte(uint8_t data);
void gpio_expander_i2c_stop(uint8_t length);

#endif // GPIO_EXPANDER_H