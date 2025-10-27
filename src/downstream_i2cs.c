#include "stdio.h"
#include "pico/stdlib.h"

#include "syzygy_mipi.h"
#include "downstream_i2cs.h"
#include "pio_i2c.h" 

static PIO pio = pio1; //pio0 used for the downstream i2c controllers


void downstream_i2c_init_all(void);
bool downstream_i2c_write(int bus_idx, uint8_t addr, const uint8_t *data, size_t len);
bool downstream_i2c_read(int bus_idx, uint8_t addr, uint8_t *data, size_t len);
void downstream_i2c_init_all(void);
void test_bus_slaves(void);

int setup_downstream_i2cs() {  
    downstream_i2c_init_all();
    // test_bus_slaves();
 
    return 0;
}

//handle Host write to downstream
void muxed_mipi_i2c_write_byte(uint8_t data) {
    
}

//Handle Host read from downstream
void muxed_mipi_i2c_read_byte(uint8_t *buffer) {
    buffer[0] = selected_mipi_device; //simply 
}

//Handle Host stop
void muxed_mipi_i2c_stop(uint8_t length) {
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


void downstream_i2c_init_all(void) {
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
    b->offset = pio_add_program(b->pio, &i2c_program);
    b->sm = pio_claim_unused_sm(b->pio, true);
    i2c_program_init(b->pio, b->sm, b->offset, sda_pin, scl_pin);
    scan_for_slave_address(bus_idx);
    b->initialized = true;
    printf("Downstream I2C bus %d initialized on pins %d (SDA), %d (SCL) with slave address 0x%02X\n", 
        bus_idx, sda_pin, scl_pin, b->slave_addr);
}

bool downstream_i2c_write(int bus_idx, uint8_t addr, const uint8_t *data, size_t len) {
    if (bus_idx < 0 || bus_idx >= NUM_MIPIS| !buses[bus_idx].initialized) return false;
    pio_i2c_bus_t *b = &buses[bus_idx];
    bool ok = true;
    pio_i2c_start(b->pio, b->sm);
    ok &= pio_i2c_write_blocking(b->pio, b->sm, addr, data, len);
    ok &= pio_i2c_read_blocking(b->pio, b->sm, addr, data, len); //ack/nack
    pio_i2c_stop(b->pio, b->sm);
    return ok;
}

bool downstream_i2c_read(int bus_idx, uint8_t addr, uint8_t *data, size_t len) {
    if (bus_idx < 0 || bus_idx >= NUM_MIPIS || !buses[bus_idx].initialized) return false;
    pio_i2c_bus_t *b = &buses[bus_idx];
    bool ok = true;
    pio_i2c_start(b->pio, b->sm);
    ok &= pio_i2c_write_blocking(b->pio, b->sm, (addr << 1) | 1, data, len);
    ok &= pio_i2c_read_blocking(b->pio, b->sm, addr, data, len);
    pio_i2c_stop(b->pio, b->sm);
    return ok;
}

/*
    Not using for now
*/
uint8_t scan_for_slave_address(int bus_idx) {
    if (bus_idx < 0 || bus_idx >= NUM_MIPIS || !buses[bus_idx].initialized) return 0xFF;
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
    for (int bus = 0; bus < NUM_MIPIS; bus++) {
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