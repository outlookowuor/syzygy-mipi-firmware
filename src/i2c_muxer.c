#include <stdio.h>
#include "hardware/i2c.h"
#include "i2c_fifo.h"
#include "i2c_slave.h"

#include "hardware/pio.h"
#include "pio_i2c.h" 
#include "syzygy_mipi.h"
#include "downstream_i2cs.h"
#include "i2c_muxer.h"
#include "i2c_multi.h"

/**
 * i2c muxer: select active MIPI between the 3 MIPIs
 * 
 * The host uses MUXED_I2C_ADDRESS to communicate with the 
 * the selected MIPI
 */
uint8_t selected_mipi_device = 0;
static void init_muxer_i2c();


static int active_bus_idx; // default bus 0


int setup_i2c_muxer() {
    init_muxer_i2c();
    return 0;
}

static void init_muxer_i2c() {
    setup_downstream_i2cs();
    selected_mipi_device = MIPI_DEVICE_0;
}

static bool toggle = false;

bool i2c_muxer_i2c_receive(uint8_t data) {
    active_bus_idx = data & 0x03;  // we are selecting active bus
    selected_mipi_device = active_bus_idx;

    toggle = !toggle;
    if (toggle) {
        printf("ACKing 0x%02x\n", data);
        return true;   // ACK
    } else {
        printf("NACKing 0x%02x\n", data);
        return false;  // NACK
    }
}

bool  i2c_muxer_i2c_restart_request(uint8_t *buffer) {
    //same as normal request
    return i2c_muxer_i2c_request(buffer);
}

void i2c_muxer_i2c_stop(uint8_t length) {


    //nothing to do
}

/** buffer is pointer to output buffer */
bool i2c_muxer_i2c_request(uint8_t *buffer) {
    buffer[0] = selected_mipi_device; //simply 
    // printf("I2C Muxer read request, sending back selected MIPI %d\n", buffer[0]);
    return true;
}

void i2c_muxer_i2c_write_read_byte(uint8_t *buffer) {
    buffer[0] = selected_mipi_device; //simply 
    // printf("I2C Muxer read request, sending back selected MIPI %d\n", buffer[0]);

}