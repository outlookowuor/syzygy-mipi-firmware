#include "stdio.h"
#include "pico/stdlib.h"

#include "syzygy_mipi.h"
#include "downstream_i2cs.h"
#include "pio_i2c.h" 

/*
 * I²C Bridge for Raspberry Pi Pico2
 * ---------------------------------------------------------
 * This implementation bridges an upstream I²C master 
 * to a downstream I²C slave 
 * 
 * Features:
 *   - Upstream side: I²C slave interface (using i2c0)
 *   - Downstream side: I²C master interface (hardware or PIO-based)
 *
 * Integration:
 *   The upstream I²C callbacks should enqueue data into the bridge’s buffers
 *   and signal events to core1 via semaphore. Core1 performs the downstream
 *   I²C transactions and prepares response data.
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "pico/sync.h"


#define NUM_MIPIS 3  // Number of downstream MIPI I2C buses


size_t data_length = 0;

void downstream_i2c_init_all(void);
bool downstream_i2c_write(int bus_idx, const uint8_t *data, size_t len);
bool downstream_i2c_read(int bus_idx, uint8_t *data, size_t len);
void downstream_i2c_init_all(void);
void test_bus_slaves(void);


int setup_downstream_i2cs() {  
    downstream_i2c_init_all();
    test_bus_slaves();

    return 0;
}

//handle Host write to downstream
bool bridge_i2c_receive(uint8_t data) {
    return downstream_i2c_write(selected_mipi_device, &data, 1);
}

//Handle Host read from downstream
bool bridge_i2c_request(uint8_t *buffer) {
    downstream_i2c_read(selected_mipi_device, buffer, 1);

    buffer[0] = selected_mipi_device; //simply , for now

    return true;
}


//Handle Host read from downstream
bool bridge_i2c_restart_request(uint8_t *buffer) {
    buffer[0] = selected_mipi_device; //simply 

    return true;
}

//Handle Host stop
void bridge_i2c_stop(uint8_t length) {
    //nothing to do
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

static pio_i2c_bus_t buses[NUM_MIPIS] = {0};

#define SDA0_PIN 12
#define SCL0_PIN 13
#define SDA1_PIN 10
#define SCL1_PIN 11
#define SDA2_PIN 8
#define SCL2_PIN 9

void init_bus(int bus_idx, uint8_t sda_pin, uint8_t scl_pin);
uint8_t scan_for_slave_address(int bus_idx);


static PIO pio; //pio0 used for the downstream i2c controllers
static uint8_t i2c_offset;

void downstream_i2c_init_all(void) {
    pio = pio1; // Use PIO1 for downstream I2C buses
    i2c_offset = pio_add_program(pio, &i2c_program);

    init_bus(0, SDA0_PIN, SCL0_PIN);
    init_bus(1, SDA1_PIN, SCL1_PIN);
    init_bus(2, SDA2_PIN, SCL2_PIN);

    printf("Downstream i2c initialized\n");
}   

void init_bus(int bus_idx, uint8_t sda_pin, uint8_t scl_pin) {
    pio_i2c_bus_t *b = &buses[bus_idx];
    
    b->pio = pio; // max is 4 buses / pio
    b->sda_pin = sda_pin;
    b->scl_pin = scl_pin;
    b->offset = i2c_offset;

    b->sm = pio_claim_unused_sm(b->pio, true);
    i2c_program_init(b->pio, b->sm, b->offset, sda_pin, scl_pin);

    scan_for_slave_address(bus_idx);
    
    b->initialized = true;
    printf("Downstream I2C bus %d initialized on pins %d (SDA), %d (SCL) with slave address 0x%02X\n", 
        bus_idx, sda_pin, scl_pin, b->slave_addr);
}

bool downstream_i2c_write(int bus_idx, const uint8_t *data, size_t len) {
    if (bus_idx < 0 || bus_idx >= NUM_MIPIS || !buses[bus_idx].initialized)
        return false;
    pio_i2c_bus_t *b = &buses[bus_idx];
    return pio_i2c_write_blocking(b->pio, b->sm, b->slave_addr, (uint8_t *)data, len) == 0;
}

bool downstream_i2c_read(int bus_idx, uint8_t *data, size_t len) {
    if (bus_idx < 0 || bus_idx >= NUM_MIPIS || !buses[bus_idx].initialized)
        return false;
    pio_i2c_bus_t *b = &buses[bus_idx];
    return pio_i2c_read_blocking(b->pio, b->sm, b->slave_addr, data, len) == 0;
}

static bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

/*
    We'll scan the bus for a slave address
*/
uint8_t scan_for_slave_address(int bus_idx) {
    if (bus_idx < 0 || bus_idx >= NUM_MIPIS || buses[bus_idx].initialized) 
        return 0xFF;
    
    printf("\nPIO I2C Bus Scan: bus [ %d ], sm [ %d ] \n", bus_idx, buses[bus_idx].sm);
    printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");
    
    pio_i2c_bus_t *b = &buses[bus_idx];
    uint8_t rxchar;
    for (uint8_t addr = 0; addr < 128; addr++) {
        if (addr % 16 == 0) {
            printf("%02x ", addr);
        }
        int result;
        if (reserved_addr(addr))
            result = -1;
        else
            result = pio_i2c_read_blocking(b->pio, b->sm, addr, NULL, 0);
            // result = pio_i2c_read_blocking_timeout(b->pio, b->sm, 
            //     addr, &rxchar, 1, 1000);

        if (result >= 0) {   
            b->slave_addr = addr;
            // return addr;
        }

        printf(result < 0 ? "." : "@");
        printf(addr % 16 == 15 ? "\n" : "  ");
    }
    return 0xFF;
}
/*
    test the three downstream buses' slaves
*/
void test_bus_slaves(){
    // probe all three buses
    uint8_t reg = 0x00, val = 0;
    for (int bus = 0; bus < NUM_MIPIS; bus++) {
        int slave_addr = buses[bus].slave_addr;
        printf("Testing bus %d: Address %d:  \n", bus, slave_addr);
        if (slave_addr == 0xFF || slave_addr == 0) {
            printf("  Skipping %d: no devices to test\n", bus);
            continue;
        }
        if (downstream_i2c_write(bus, &reg, 1) &&
            downstream_i2c_read(bus, &val, 1)) {
            printf("  Bus %d: device 0x68 responded, val=0x%02X\n", bus, val);
        } else {
            printf("  Bus %d: no response or read failed\n", bus);
        }
    }
}