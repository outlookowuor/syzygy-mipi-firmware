#include <stdio.h>

#include "i2c_multi.h"
#include "pico/stdlib.h"

#include "host_slave_i2cs.h"
#include "i2c_muxer.h"
#include "gpio_expander.h"
#include "clock_programmer.h"
#include "downstream_i2cs.h"


PIO pio = pio0; // First PIO is for multi-slave i2c

uint8_t buffer[64] = {0};
uint8_t latest_i2c_address = I2C_MUXER_I2C_ADDRESS;

void host_i2c_receive_handler(uint8_t data, bool is_address);
void host_i2c_request_handler(uint8_t address);
void host_i2c_stop_handler(uint8_t length);

/**
 * Initialize the 3 i2c slaves
 * - i2c_muxer
 * - gpio_expander
 * - clock_programmer
 */
void  setup_host_i2cs() {
    i2c_multi_init(pio, HOST_SDA_PIN);

    i2c_multi_enable_address(I2C_MUXER_I2C_ADDRESS);
    i2c_multi_enable_address(GPIO_EXPANDER_I2C_ADDRESS);
    i2c_multi_enable_address(CLOCK_PROGRAMMER_I2C_ADDRESS);
    i2c_multi_enable_address(MUXED_MIPI_I2C_ADDRESS);

    i2c_multi_set_receive_handler(host_i2c_receive_handler);
    i2c_multi_set_request_handler(host_i2c_request_handler);
    i2c_multi_set_stop_handler(host_i2c_stop_handler);

    i2c_multi_set_write_buffer(buffer);
}

#define HOST_SCL_PIN HOST_SDA_PIN + 1
// Make sure you include the right headers for gpio_*
static inline void scl_stretch_begin() {
    // Drive SCL low (force output low) — this stretches the clock
    gpio_put(HOST_SCL_PIN, 0);
    gpio_set_dir(HOST_SCL_PIN, GPIO_OUT);
}

static inline void scl_stretch_end() {
    // Release SCL (high-impedance) — let pull-ups raise the line
    gpio_set_dir(HOST_SCL_PIN, GPIO_IN);
    // No need to gpio_put(1); keep it controlled by pull-up
}


/* I2C received a byte: A write from master */
void handle_receive(uint8_t data){
    switch (latest_i2c_address) {
    case I2C_MUXER_I2C_ADDRESS: // I2C_MUXER write
        i2c_muxer_i2c_write_byte(data); 
        break;
    case GPIO_EXPANDER_I2C_ADDRESS: //GPIO Expander
        gpio_expander_i2c_write_byte(data);
        break;
    case CLOCK_PROGRAMMER_I2C_ADDRESS: //Clock Programmer
        clock_programmer_i2c_write_byte(data);
        break;    
    case MUXED_MIPI_I2C_ADDRESS: //Downstream i2c
        muxed_mipi_i2c_write_byte(data);
        break;
    }
}

/* I2C received a read request from master: reading a byte */
void handle_request(uint8_t i2c_address){
    switch (i2c_address) {
    case I2C_MUXER_I2C_ADDRESS: // I2C_MUXER write
        i2c_muxer_i2c_read_byte(buffer);
        break;
    case GPIO_EXPANDER_I2C_ADDRESS: //GPIO Expander
        gpio_expander_i2c_read_byte(buffer);
        break;
    case CLOCK_PROGRAMMER_I2C_ADDRESS://Clock Programmer
        clock_programmer_i2c_read_byte(buffer);
        break;
    case MUXED_MIPI_I2C_ADDRESS: // downstream I2C
        //this might take some time.. stretch the clock
        // scl_stretch_begin();
        muxed_mipi_i2c_read_byte(buffer);
        // scl_stretch_end();
        break;
    }
}

/* I2C received a stop request from master */
void handle_stop(uint8_t length){
    switch (latest_i2c_address) {
    case I2C_MUXER_I2C_ADDRESS: 
        i2c_muxer_i2c_stop(length);
        break;
    case GPIO_EXPANDER_I2C_ADDRESS:
        gpio_expander_i2c_stop(length);
        break;
    case CLOCK_PROGRAMMER_I2C_ADDRESS:
        clock_programmer_i2c_stop(length);
        break;
    case MUXED_MIPI_I2C_ADDRESS: //Clock Programmer
        muxed_mipi_i2c_stop(length);
        break;
    }
}

void host_i2c_receive_handler(uint8_t data, bool is_address) {
    if (is_address){
        //toggle addresses
        latest_i2c_address = data;
    }
    else{
        //i2c writing into a reg
        handle_receive(data);
    }
}

void host_i2c_request_handler(uint8_t address) {
    handle_receive(address);
    buffer[0] = 0x70;
    buffer[1] = 0x71;
    buffer[2] = 0x72;
    //printf("\nAddress: %X, request...", address);
}

void host_i2c_stop_handler(uint8_t length) { 
    // printf("\nTotal bytes: %u", length); 
    handle_stop(length);
    printf("\n[ %02X ]: Total bytes: %u", latest_i2c_address, length); 
}