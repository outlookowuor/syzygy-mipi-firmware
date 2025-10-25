#ifndef GPIO_EXPANDER_H
#define GPIO_EXPANDER_H

#define GPIO_EXPANDER_I2C_ADDRESS 0x50

#define GPIO_18 18  // MIPI_00
#define GPIO_19 19  // MIPI_10
#define GPIO_20 20  // MIPI_20
#define GPIO_23 23  // MIPI_01
#define GPIO_24 24  // MIPI_11
#define GPIO_25 25  // MIPI_21

#define GPIO_HOST 26

int setup_gpio_expander();

void gpio_expander_i2c_write_byte(uint8_t data);
void gpio_expander_i2c_read_byte(uint8_t* buffer);
void gpio_expander_i2c_stop(uint8_t length);

#endif // GPIO_EXPANDER_H