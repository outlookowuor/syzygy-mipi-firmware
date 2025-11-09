#include <stdio.h>

#include "i2c_multi.h"
#include "pico/stdlib.h"

#include "host_slave_i2cs.h"
#include "i2c_muxer.h"
#include "gpio_expander.h"
#include "clock_programmer.h"
#include "downstream_i2cs.h"


static PIO pio = pio0; // First PIO is for multi-slave i2c

bool host_i2c_receive_handler(uint8_t data, bool is_address);
bool host_i2c_request_handler(uint8_t address);
void host_i2c_stop_handler(uint8_t length);
bool host_i2c_restart_request_handler(uint8_t data);

static uint8_t buffer[64] = {0};
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



static uint8_t latest_i2c_address = I2C_MUXER_I2C_ADDRESS;

/* I2C received a byte: a write from master */
bool handle_receive(uint8_t data){
    bool ok = true;
    switch (latest_i2c_address) {
    case I2C_MUXER_I2C_ADDRESS: // I2C_MUXER write
        ok = i2c_muxer_i2c_receive(data); 
        break;
    case GPIO_EXPANDER_I2C_ADDRESS: //GPIO Expander
        ok = gpio_expander_i2c_receive(data);
        break;
    case CLOCK_PROGRAMMER_I2C_ADDRESS: //Clock Programmer
        ok = clock_programmer_i2c_receive(data);
        break;    
    case MUXED_MIPI_I2C_ADDRESS: //Downstream i2c
        ok = bridge_i2c_receive(data);
        break;
    }

    return ok;
}

/* I2C received a read request from master: reading a byte */
bool handle_request(uint8_t i2c_address){
    bool ok = true;
    switch (i2c_address) {
    case I2C_MUXER_I2C_ADDRESS: // I2C_MUXER write
        ok = i2c_muxer_i2c_request(buffer);
        break;
    case GPIO_EXPANDER_I2C_ADDRESS: //GPIO Expander
        ok = gpio_expander_i2c_request(buffer);
        break;
    case CLOCK_PROGRAMMER_I2C_ADDRESS://Clock Programmer
        ok = clock_programmer_i2c_request(buffer);
        break;
    case MUXED_MIPI_I2C_ADDRESS: // downstream I2C
        ok = bridge_i2c_request(buffer);
        break;
    }

    return ok;
}


/* I2C received a read request from master without STOP 
    of previous transaction: reading a byte */
bool handle_restart(uint8_t i2c_address){
    bool ok  = true;
    switch (i2c_address) {
    case I2C_MUXER_I2C_ADDRESS: // I2C_MUXER write
        ok = i2c_muxer_i2c_restart_request(buffer);
        break;
    case GPIO_EXPANDER_I2C_ADDRESS: //GPIO Expander
        ok = gpio_expander_i2c_restart_request(buffer);
        break;
    case CLOCK_PROGRAMMER_I2C_ADDRESS://Clock Programmer
        ok = clock_programmer_i2c_restart_request(buffer);
        break;
    case MUXED_MIPI_I2C_ADDRESS: // downstream I2C
        ok = bridge_i2c_restart_request(buffer);
        break;
    }

    return ok;
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
        bridge_i2c_stop(length);
        break;
    }
}


static bool pending_restart = false;  //to monitor restart condition

bool host_i2c_receive_handler(uint8_t data, bool is_address) {
    bool ok = true;
    if (is_address){
        latest_i2c_address = data;
    }
    else{
        ok = handle_receive(data);
    }
    pending_restart = true;

    return ok;
}

bool host_i2c_request_handler(uint8_t address) {
    bool ok = true;
    if (pending_restart) { 
        // request arrived before a stop`
        ok = host_i2c_restart_request_handler(address);
        pending_restart = false;
    }
    else {
        ok = handle_request(address);
    }

    return ok;
}

void host_i2c_stop_handler(uint8_t length) { 
    // printf("\nTotal bytes: %u", length); 
    handle_stop(length);
    pending_restart = false;

    printf("\n[ %02X ]: Total bytes: %u", latest_i2c_address, length); 
}

bool host_i2c_restart_request_handler(uint8_t data){ 
    //handle start/restart conditions if needed
    return handle_restart(data);
}