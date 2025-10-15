#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "i2c_fifo.h"
#include "i2c_slave.h"

#include "hardware/pio.h"
#include "pio_i2c.h" 

#include "syzygy_mipi.h"

#include "i2c_muxer.h"


/**
 * i2c muxer imitates a TCA9544A I2C multiplexer with 3 channels
 * 
 * 
 * Create 3 x I2C controllers using PIO and management buffers
 * Is it multiplexing or simply shuffling between multiple I2C controllers?
 * 
 * When BYPASSn is High, 
 * Data (byte?, SDA wire state?) from the host i2c (slave) is relayed (buffered?
 * PIO code ?) to the currently "active"/"selected" master I2C (on to its slave)
 * 
 * Need to think through the return path. 
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

    // Initialize the I2C muxer slave interface
    init_muxer_i2c();
    printf("I2C slave ready at 0x%02X\n", MUXER_ADDRESS);

    // Initialize downstream I2C controllers
    downstream_i2c_init_all();
    printf("Downstream i2c initialized\n");

    //test slaves on all buses
    test_bus_slaves();

        
    return 0;
}



#define CMD_SELECT  0x00
#define CMD_WRITE   0x01
#define CMD_READ    0x02

#define MAX_BUF 64

static uint8_t rx_buf[MAX_BUF];
static int rx_len = 0;

static uint8_t tx_buf[MAX_BUF];
static int tx_len = 0;
static int tx_index = 0;


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
static void init_muxer_i2c() {
    gpio_init(MUXER_I2C_SDA_PIN);
    gpio_set_function(MUXER_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(MUXER_I2C_SDA_PIN);

    gpio_init(MUXER_I2C_SCL_PIN);
    gpio_set_function(MUXER_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(MUXER_I2C_SCL_PIN);

    i2c_init(i2c1, I2C_BAUDRATE);
    // configure I2C0 for slave mode
    
    i2c_slave_init(i2c1, MUXER_ADDRESS, &i2c_slave_handler);
}

/*
Implementation of command-driven muxing
| Byte | Meaning                                                                                                |
| ---- | ------------------------------------------------------------------------------------------------------ |
| 0x00 | Set active downstream bus (bits 0–2 mask)                                                              |
| 0x01 | Write transaction: `[0x01, bus, addr, len, data…]`                                                     |
| 0x02 | Read transaction: `[0x02, bus, addr, len]` followed by a master read from 0x70 to get `len` bytes back |
*/

static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    switch (event) {
    case I2C_SLAVE_RECEIVE: // master has written some data
        if (rx_len < MAX_BUF)
                rx_buf[rx_len++] = i2c_read_byte(i2c);
            break;
        break;
        
    case I2C_SLAVE_REQUEST: // master is requesting data
        if (tx_index < tx_len)
            i2c_write_byte(i2c, tx_buf[tx_index++]);
        else
            i2c_write_byte(i2c, 0xFF); 
        break;
    case I2C_SLAVE_FINISH: // master has signalled Stop / Restart
        if (rx_len == 0) break;

        uint8_t cmd = rx_buf[0];
        switch (cmd) {
            case CMD_SELECT:
                if (rx_len > 1) {
                    active_bus_idx = rx_buf[1];
                    printf("Bus selected: %u\n", active_bus);
                }
                break;

            case CMD_WRITE:
                if (rx_len >= 4) {
                    uint8_t bus = rx_buf[1];
                    uint8_t addr = rx_buf[2];
                    uint8_t len = rx_buf[3];
                    downstream_i2c_write(bus, addr, &rx_buf[4], len);
                    printf("Wrote %dB to 0x%02X on bus%d\n", len, addr, bus);
                }
                break;

            case CMD_READ:
                if (rx_len >= 4) {
                    uint8_t bus = rx_buf[1];
                    uint8_t addr = rx_buf[2];
                    uint8_t len = rx_buf[3];
                    tx_len = downstream_i2c_read(bus, addr, tx_buf, len);
                    tx_index = 0;
                    printf("Prepared %dB read from 0x%02X on bus%d\n", tx_len, addr, bus);
                }
                break;

            default:
                printf("Unknown CMD 0x%02X\n", cmd);
                break;
        }
        rx_len = 0;

        break;
    default:
        break;
    }
}



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

bool downstream_i2c_write(int bus_idx, uint8_t addr, const uint8_t *data, size_t len) {
    if (bus_idx < 0 || bus_idx >= NUM_CHANNELS| !buses[bus_idx].initialized) return false;
    pio_i2c_bus_t *b = &buses[bus_idx];
    bool ok = true;
    ok &= pio_i2c_start(b->pio, b->sm);
    ok &= pio_i2c_write_byte(b->pio, b->sm, (addr << 1) | 0);
    for (size_t i = 0; i < len; i++)
        ok &= pio_i2c_write_byte(b->pio, b->sm, data[i]);
    ok &= pio_i2c_stop(b->pio, b->sm);
    return ok;
}

bool downstream_i2c_read(int bus_idx, uint8_t addr, uint8_t *data, size_t len) {
    if (bus_idx < 0 || bus_idx >= NUM_CHANNELS || !buses[bus_idx].initialized) return false;
    pio_i2c_bus_t *b = &buses[bus_idx];
    bool ok = true;
    ok &= pio_i2c_start(b->pio, b->sm);
    ok &= pio_i2c_write_byte(b->pio, b->sm, (addr << 1) | 1);
    for (size_t i = 0; i < len; i++)
        data[i] = pio_i2c_read_byte(b->pio, b->sm, i < (len - 1));
    ok &= pio_i2c_stop(b->pio, b->sm);
    return ok;
}