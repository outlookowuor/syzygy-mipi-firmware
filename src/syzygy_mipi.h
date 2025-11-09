#ifndef _SYZYGY_MIPI_H_
#define _SYZYGY_MIPI_H_



#define NUM_MIPIS 3

#define MIPI_DEVICE_0 0
#define MIPI_DEVICE_1 1
#define MIPI_DEVICE_2 2

extern uint8_t selected_mipi_device; 

#define DEFAULT_MUXER_ADDRESS 0x70
#define DEFAULT_GPIO_EXPANDER_ADDRESS 0x50
#define DEFAULT_CLOCK_PROGRAMMER_ADDRESS 0x17
#define DEFAULT_BRIDGE_ADDRESS 0x72

extern uint8_t i2c_muxer_address;
extern uint8_t gpio_expander_address;
extern uint8_t clock_programmer_address;
extern uint8_t bridge_address;

bool is_valid_mipi(uint8_t mipi_idx);

#endif