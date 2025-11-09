#include <stdio.h>

#include "i2c_multi.h"
#include "pico/stdlib.h"

#include "host_slave_i2cs.h"
#include "i2c_muxer.h"
#include "gpio_expander.h"
#include "clock_programmer.h"
#include "downstream_i2cs.h"
#include "configs.h"


static PIO pio = pio0; // First PIO is for multi-slave i2c

void host_i2c_receive_handler(uint8_t data, bool is_address);
void host_i2c_request_handler(uint8_t address);
void host_i2c_stop_handler(uint8_t length);
void host_i2c_restart_request_handler(uint8_t data);

static uint8_t buffer[64] = {0};
/**
 * Initialize the 3 i2c slaves
 * - i2c_muxer
 * - gpio_expander
 * - clock_programmer
 */
void  setup_host_i2cs() {
    i2c_multi_init(pio, HOST_SDA_PIN);

    i2c_multi_enable_address(config.muxer_addr);
    i2c_multi_enable_address(config.gpio_expander_addr);
    i2c_multi_enable_address(config.clock_programmer_addr);
    i2c_multi_enable_address(config.bridge_addr);

    i2c_multi_set_receive_handler(host_i2c_receive_handler);
    i2c_multi_set_request_handler(host_i2c_request_handler);
    i2c_multi_set_stop_handler(host_i2c_stop_handler);

    i2c_multi_set_write_buffer(buffer);
}

static uint8_t latest_i2c_address; //default address

/* I2C received a byte: a write from master */
void handle_receive(uint8_t data){
    if (latest_i2c_address == config.muxer_addr) {
        i2c_muxer_i2c_receive(data);
    } else if (latest_i2c_address == config.gpio_expander_addr) {
        gpio_expander_i2c_receive(data);
    } else if (latest_i2c_address == config.clock_programmer_addr) {
        clock_programmer_i2c_receive(data);
    } else if (latest_i2c_address == config.bridge_addr) {
        bridge_i2c_receive(data);
    }
}

/* I2C received a read request from master: reading a byte */
void handle_request(uint8_t i2c_address){
    if (i2c_address == config.muxer_addr) { // I2C_MUXER write
        i2c_muxer_i2c_request(buffer);
    } else if (i2c_address == config.gpio_expander_addr) { //GPIO Expander
        gpio_expander_i2c_request(buffer);
    } else if (i2c_address == config.clock_programmer_addr) { //Clock Programmer
        clock_programmer_i2c_request(buffer);
    } else if (i2c_address == config.bridge_addr) { // downstream I2C
        bridge_i2c_request(buffer);
    }
}


/* I2C received a read request from master without STOP 
    of previous transaction: reading a byte */
void handle_restart(uint8_t i2c_address){
    if (i2c_address == config.muxer_addr) { // I2C_MUXER write
        i2c_muxer_i2c_restart_request(buffer);
    } else if (i2c_address == config.gpio_expander_addr) { //GPIO Expander
        gpio_expander_i2c_restart_request(buffer);
    } else if (i2c_address == config.clock_programmer_addr) { //Clock Programmer
        clock_programmer_i2c_restart_request(buffer);
    } else if (i2c_address == config.bridge_addr) { // downstream I2C 
        bridge_i2c_restart_request(buffer);
    }
}


/* I2C received a stop request from master */
void handle_stop(uint8_t length){
    if (latest_i2c_address == config.muxer_addr) {
        i2c_muxer_i2c_stop(length);
    } else if (latest_i2c_address == config.gpio_expander_addr) {
        gpio_expander_i2c_stop(length);
    } else if (latest_i2c_address == config.clock_programmer_addr) {
        clock_programmer_i2c_stop(length);
    } else if (latest_i2c_address == config.bridge_addr) {
        bridge_i2c_stop(length);
    }
}


static bool pending_restart = false;  //to monitor restart condition

void host_i2c_receive_handler(uint8_t data, bool is_address) {
    if (is_address){
        latest_i2c_address = data;
    }
    else{
        handle_receive(data);
    }
    pending_restart = true;
}

void host_i2c_request_handler(uint8_t address) {
    if (pending_restart) { 
        // request arrived before a stop`
        host_i2c_restart_request_handler(address);
        pending_restart = false;
    }
    else {
        handle_request(address);
    }
}

void host_i2c_stop_handler(uint8_t length) { 
    // printf("\nTotal bytes: %u", length); 
    handle_stop(length);
    pending_restart = false;

    printf("\n[ %02X ]: Total bytes: %u", latest_i2c_address, length); 
}

void host_i2c_restart_request_handler(uint8_t data){ 
    //handle start/restart conditions if needed
    handle_restart(data);
}