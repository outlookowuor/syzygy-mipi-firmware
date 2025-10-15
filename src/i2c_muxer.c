#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "i2c_fifo.h"
#include "i2c_slave.h"

#include "hardware/pio.h"
#include "pio_i2c.h" 
#include "syzygy_mipi.h"
#include "i2c_muxer.h"
#include "i2c_multi.h"


/**
 * i2c muxer attempts imitates a TCA9544A I2C multiplexer with 3 channels
 * 
 * Using multi-slave i2c on pio1 as the slave interface to the
 * like taca9544a, it accepts a single byte to select one of 3 downstream
 * 
 * Create 3 x I2C controllers using PIO 
 * 
 * When BYPASSn is High, 
 * Data (byte?, SDA wire state?) from the host i2c (slave) is relayed (buffered?
 * PIO code ?) to the currently "active"/"selected" master I2C (on to its slave)
 * 
 */

#define MUXER_I2C_SDA_PIN 2
#define  MUXER_I2C_SCL_PIN 3
#define I2C_BAUDRATE 100000

#define NUM_CHANNELS 3
#define MUXER_ADDRESS 0x70 // 0x70, 0x71, 0x72


#ifndef uint8_t
typedef unsigned char uint8_t;
#endif
#ifndef size_t
typedef unsigned int size_t;
#endif
#ifndef uint 
typedef unsigned int uint;
#endif


static volatile uint8_t active_channel_mask = 0x01;  // default channel 0


static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event);
static void init_muxer_i2c();

bool downstream_i2c_write(int bus_idx, uint8_t addr, const uint8_t *data, size_t len);
bool downstream_i2c_read(int bus_idx, uint8_t addr, uint8_t *data, size_t len);
void downstream_i2c_init_all(void);
void test_bus_slaves(void);

static int active_bus_idx = 0; // default bus 0



int setup_i2c_muxer() {

    // Initialize the I2C muxer slave multi interface
    init_muxer_i2c();
    printf("I2C slave ready at 0x%02X\n", MUXER_ADDRESS);

    // Initialize downstream I2C controllers
    downstream_i2c_init_all();
    printf("Downstream i2c initialized\n");

    //test slaves on all buses
    test_bus_slaves();

        
    return 0;
}



/**
 * Downstream I2C controllers using PIO
 */
typedef struct {
    PIO pio;
    uint sm;
    uint8_t sda_pin;
    uint8_t scl_pin;
    uint8_t offset;
    bool initialized;
    uint8_t slave_addr;
} pio_i2c_bus_t;

static pio_i2c_bus_t buses[NUM_CHANNELS] = {0};


PIO pio = pio1; //pio0 used for the downstream i2c controllers
uint pin = MUXER_I2C_SDA_PIN;
uint8_t buffer[64] = {0};

bool muxer_conversation_in_progress = false;

 

void i2c_receive_handler(uint8_t data, bool is_address);
void i2c_request_handler(uint8_t address);
void i2c_stop_handler(uint8_t length);


void  init_muxer_i2c() {
    stdio_init_all();
    i2c_multi_init(pio, pin);
    i2c_multi_enable_all_addresses();
    i2c_multi_set_receive_handler(i2c_receive_handler);
    i2c_multi_set_request_handler(i2c_request_handler);
    i2c_multi_set_stop_handler(i2c_stop_handler);
    i2c_multi_set_write_buffer(buffer);
}

void i2c_receive_handler(uint8_t data, bool is_address) {
    if (is_address  & data==MUXER_ADDRESS) {
        muxer_conversation_in_progress = true;
        return;
    }      
    if (muxer_conversation_in_progress == true){
        //just one byte to select active bus
        active_bus_idx = data & 0x03; // only 3 channels
        active_channel_mask = 1 << active_bus_idx;
        //end the muxer conversation
        muxer_conversation_in_progress = false;
        return;
    }  
    // treat as a write to slave address
    downstream_i2c_write(active_bus_idx, data, NULL, 0); 
}

void i2c_request_handler(uint8_t address) {
    // treat as a read from slave address
    if (address != MUXER_ADDRESS) {
        downstream_i2c_read(active_bus_idx, address, buffer, 1);
    }
    else {
        // return the active channel mask
        buffer[0] = buses[0].slave_addr;
        buffer[1] = buses[1].slave_addr;
        buffer[2] = buses[2].slave_addr;
    }

}

void i2c_stop_handler(uint8_t length) { 
    muxer_conversation_in_progress = false;
}


#define SDA0_PIN 12
#define SCL0_PIN 13
#define SDA1_PIN 10
#define SCL1_PIN 11
#define SDA2_PIN 8
#define SCL2_PIN 9

void init_bus(int bus_idx, uint8_t sda_pin, uint8_t scl_pin);
uint8_t scan_for_slave_address(int bus_idx);


void downstream_i2c_init_all(void) {
    init_bus(0, SDA0_PIN, SCL0_PIN);
    init_bus(1, SDA1_PIN, SCL1_PIN);
    init_bus(2, SDA2_PIN, SCL0_PIN);
}

void init_bus(int bus_idx, uint8_t sda_pin, uint8_t scl_pin) {
    pio_i2c_bus_t *b = &buses[bus_idx];
    
    b->pio = pio0; // max is 4 buses / pio
    b->sda_pin = sda_pin;
    b->scl_pin = scl_pin;
    b->offset = pio_add_program(b->pio, &i2c_program);
    b->sm = pio_claim_unused_sm(b->pio, true);
    i2c_program_init(b->pio, b->sm, b->offset, sda_pin, scl_pin);
    scan_for_slave_address(bus_idx);
    b->initialized = true;
    printf("Downstream I2C bus %d initialized on pins %d (SDA), %d (SCL) with slave address 0x%02X\n", 
        bus_idx, sda_pin, scl_pin, b->slave_addr);
}

bool downstream_i2c_write(int bus_idx, uint8_t addr, const uint8_t *data, size_t len) {
    if (bus_idx < 0 || bus_idx >= NUM_CHANNELS| !buses[bus_idx].initialized) return false;
    pio_i2c_bus_t *b = &buses[bus_idx];
    bool ok = true;
    pio_i2c_start(b->pio, b->sm);
    ok &= pio_i2c_write_blocking(b->pio, b->sm, addr, data, len);
    ok &= pio_i2c_read_blocking(b->pio, b->sm, addr, data, len); //ack/nack
    pio_i2c_stop(b->pio, b->sm);
    return ok;
}

bool downstream_i2c_read(int bus_idx, uint8_t addr, uint8_t *data, size_t len) {
    if (bus_idx < 0 || bus_idx >= NUM_CHANNELS || !buses[bus_idx].initialized) return false;
    pio_i2c_bus_t *b = &buses[bus_idx];
    bool ok = true;
    pio_i2c_start(b->pio, b->sm);
    ok &= pio_i2c_write_blocking(b->pio, b->sm, (addr << 1) | 1, data, len);
    ok &= pio_i2c_read_blocking(b->pio, b->sm, addr, data, len);
    pio_i2c_stop(b->pio, b->sm);
    return ok;
}

uint8_t scan_for_slave_address(int bus_idx) {
    if (bus_idx < 0 || bus_idx >= NUM_CHANNELS || !buses[bus_idx].initialized) return 0xFF;
    pio_i2c_bus_t *b = &buses[bus_idx];
    for (uint8_t addr = 1; addr < 128; addr++) {
        int dummy = pio_i2c_read_blocking(b->pio, b->sm, addr, NULL, 0);
        if (dummy >= 0) {   
            b->slave_addr = addr;
            return addr;
        }
    }
    return 0xFF;
}
/*
    test the three downstream buses' slaves
*/
void test_bus_slaves(){
    // probe all three buses
    uint8_t reg = 0x00, val = 0;
    for (int bus = 0; bus < NUM_CHANNELS; bus++) {
        int slave_addr = buses[bus].slave_addr;
        printf("Testing bus %d: Address %d:  \n", bus, slave_addr);
        if (slave_addr == 0xFF || slave_addr == 0) {
            printf("  Skipping %d: no devices to test\n", bus);
            continue;
        }
        if (downstream_i2c_write(bus, slave_addr, &reg, 1) &&
            downstream_i2c_read(bus, slave_addr, &val, 1)) {
            printf("  Bus %d: device 0x68 responded, val=0x%02X\n", bus, val);
        } else {
            printf("  Bus %d: no response or read failed\n", bus);
        }
    }
}