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
 *   - Core1 handles downstream operations asynchronously
 *   - Buffered forwarding with STOP-triggered send
 *   - Optional clock stretching for downstream reads
 *
 * Integration:
 *   The upstream I²C callbacks should enqueue data into the bridge’s buffers
 *   and signal events to core1 via semaphore. Core1 performs the downstream
 *   I²C transactions and prepares response data.
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "pico/sync.h"

// ================= CONFIGURATION =================
#define UPSTREAM_I2C        i2c0
#define UPSTREAM_SDA_PIN    4
#define UPSTREAM_SCL_PIN    5
#define UPSTREAM_SLAVE_ADDR 0x42

#define DOWNSTREAM_I2C      i2c1
#define DOWNSTREAM_SDA_PIN  6
#define DOWNSTREAM_SCL_PIN  7
#define DOWNSTREAM_TARGET_ADDR 0x50

#define BUF_SZ 256
#define RESP_SZ 256
#define CLOCK_STRETCH_FOR_READS 1

// ================= TYPES =================
typedef struct {
    uint8_t buf[BUF_SZ];
    size_t len;
} i2c_txbuf_t;

typedef struct {
    uint8_t buf[RESP_SZ];
    size_t len;
    bool ready;
} i2c_rxbuf_t;

// ================= GLOBALS =================
static i2c_txbuf_t tx_buf;
static i2c_rxbuf_t rx_buf;
static volatile bool pending_downstream;
static volatile bool expect_read;
static semaphore_t downstream_sem;
static mutex_t buf_mutex;


void downstream_i2c_init_all(void);
bool downstream_i2c_write(int bus_idx, uint8_t addr, const uint8_t *data, size_t len);
bool downstream_i2c_read(int bus_idx, uint8_t addr, uint8_t *data, size_t len);
void downstream_i2c_init_all(void);
void test_bus_slaves(void);


// ================= CORE1 WORKER =================
static void core1_worker() {
    while (true) {
        sem_acquire_blocking(&downstream_sem);
        mutex_enter_blocking(&buf_mutex);

        if (!pending_downstream) {
            mutex_exit(&buf_mutex);
            continue;
        }

        size_t wlen = tx_buf.len;
        uint8_t tmp[BUF_SZ];
        memcpy(tmp, tx_buf.buf, wlen);
        bool rd = expect_read;
        tx_buf.len = 0;
        pending_downstream = false;
        mutex_exit(&buf_mutex);

        if (!downstream_i2c_write(selected_mipi_device, DOWNSTREAM_TARGET_ADDR, tmp, wlen)) {
            printf("[core1] Downstream write failed\n");
            continue;
        }

        if (rd) {
            mutex_enter_blocking(&buf_mutex);
            rx_buf.len = RESP_SZ;
            rx_buf.ready = downstream_i2c_read(selected_mipi_device, DOWNSTREAM_TARGET_ADDR, rx_buf.buf, rx_buf.len);
            expect_read = false;
            mutex_exit(&buf_mutex);
        }
    }
}

// ================= UPSTREAM CALLBACK HANDLERS =================
void bridge_on_write(uint8_t byte) {
    if (tx_buf.len < BUF_SZ)
        tx_buf.buf[tx_buf.len++] = byte;
}

void bridge_on_stop() {
    mutex_enter_blocking(&buf_mutex);
    pending_downstream = true;
    expect_read = false;
    mutex_exit(&buf_mutex);
    sem_release(&downstream_sem);
}

void bridge_on_repeated_start(bool next_is_read) {
    if (next_is_read) {
        mutex_enter_blocking(&buf_mutex);
        expect_read = true;
        pending_downstream = true;
        mutex_exit(&buf_mutex);
        sem_release(&downstream_sem);
    }
}

uint8_t bridge_on_read() {
#if CLOCK_STRETCH_FOR_READS
    gpio_set_dir(UPSTREAM_SCL_PIN, GPIO_OUT);
    gpio_put(UPSTREAM_SCL_PIN, 0);
#endif

    uint8_t val = 0xFF;
    mutex_enter_blocking(&buf_mutex);
    if (rx_buf.ready && rx_buf.len > 0) {
        val = rx_buf.buf[RESP_SZ - rx_buf.len];
        rx_buf.len--;
        if (rx_buf.len == 0) rx_buf.ready = false;
    }
    mutex_exit(&buf_mutex);

#if CLOCK_STRETCH_FOR_READS
    gpio_set_dir(UPSTREAM_SCL_PIN, GPIO_IN);
#endif
    return val;
}

// ================= INITIALIZATION =================
void bridge_init() {
    mutex_init(&buf_mutex);
    sem_init(&downstream_sem, 0, 1);

    tx_buf.len = 0;
    rx_buf.len = 0;
    rx_buf.ready = false;
    pending_downstream = false;
    expect_read = false;
                                                                                                
    i2c_init(DOWNSTREAM_I2C, 400000);
    gpio_set_function(DOWNSTREAM_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(DOWNSTREAM_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(DOWNSTREAM_SDA_PIN);
    gpio_pull_up(DOWNSTREAM_SCL_PIN);                                                                                                                                                                           

    multicore_launch_core1(core1_worker);
}

/*
Example usage in your main program:

int main() {
    stdio_init_all();
    bridge_init();
    setup_upstream_slave_callbacks(bridge_on_write, bridge_on_read, bridge_on_stop, bridge_on_repeated_start);
    while (true) tight_loop_contents();
}
*/


int setup_downstream_i2cs() {  
    downstream_i2c_init_all();
    // test_bus_slaves();
 
    return 0;
}

//handle Host write to downstream
void bridge_i2c_receive(uint8_t data) {
    
}

//Handle Host read from downstream
void bridge_i2c_request(uint8_t *buffer) {
    buffer[0] = selected_mipi_device; //simply 
}


//Handle Host read from downstream
void bridge_i2c_restart_request(uint8_t *buffer) {
    buffer[0] = selected_mipi_device; //simply 
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


bool pio_i2c_try_read_byte(PIO pio, uint sm, uint8_t *val) {
    if (pio_sm_is_rx_fifo_empty(pio, sm))
        return false; // nothing yet
    *val = pio_sm_get(pio, sm) >> 24; // same as normal read
    return true;
}


bool pio_i2c_read_blocking_timeout(PIO pio, uint sm,
                                   uint8_t addr, uint8_t *rxbuf, size_t len,
                                   uint timeout_us)
{
    absolute_time_t deadline = make_timeout_time_us(timeout_us);
    bool ok = true;

    // Start the read
    pio_i2c_start(pio, sm);

    // Send address + read bit
    if (!pio_i2c_write_blocking(pio, sm, (addr << 1) | 1, rxbuf, len)) {
        ok = false;
        goto abort;
    }

    for (size_t i = 0; i < len; i++) {
        // Check timeout each iteration
        if (absolute_time_diff_us(deadline, get_absolute_time()) < 0) {
            ok = false;
            goto abort;
        }

        // Read one byte (if it hangs, we'll break out on next loop)
        if (!pio_i2c_try_read_byte(pio, sm, &rxbuf[i])) {
            // Optional small sleep to yield
            sleep_us(10);
        }
    }

abort:
    pio_i2c_stop(pio, sm);

    if (!ok) {
        // Reset the state machine to clear any stuck conditions
        pio_sm_set_enabled(pio, sm, false);
        pio_sm_restart(pio, sm);
        pio_sm_clear_fifos(pio, sm);
        pio_sm_set_enabled(pio, sm, true);
    }

    return ok;
}

bool downstream_i2c_write(int bus_idx, uint8_t addr, const uint8_t *data, size_t len) {
    if (bus_idx < 0 || bus_idx >= NUM_MIPIS || !buses[bus_idx].initialized)
        return false;
    pio_i2c_bus_t *b = &buses[bus_idx];
    return pio_i2c_write_blocking(b->pio, b->sm, addr, (uint8_t *)data, len) == 0;
}

bool downstream_i2c_read(int bus_idx, uint8_t addr, uint8_t *data, size_t len) {
    if (bus_idx < 0 || bus_idx >= NUM_MIPIS || !buses[bus_idx].initialized)
        return false;
    pio_i2c_bus_t *b = &buses[bus_idx];
    return pio_i2c_read_blocking(b->pio, b->sm, addr, data, len) == 0;
}


// const int PIO_I2C_DATA_LSB   = 1;
// const int PIO_I2C_FINAL_LSB  = 9;

// int pio_i2c_write_blocking_nostop(PIO pio, uint sm, uint8_t addr, uint8_t *txbuf, uint len) {
//     int err = 0;
//     pio_i2c_start(pio, sm);
//     pio_i2c_rx_enable(pio, sm, false);
//     pio_i2c_put16(pio, sm, (addr << 2) | 1u);
//     while (len && !pio_i2c_check_error(pio, sm)) {
//         if (!pio_sm_is_tx_fifo_full(pio, sm)) {
//             --len;
//             pio_i2c_put_or_err(pio, sm, (*txbuf++ << PIO_I2C_DATA_LSB) | ((len == 0) << PIO_I2C_FINAL_LSB) | 1u);
//         }
//     }
//     pio_i2c_wait_idle(pio, sm);
//     if (pio_i2c_check_error(pio, sm)) {
//         err = -1;
//         pio_i2c_resume_after_error(pio, sm);
//         pio_i2c_stop(pio, sm);
//     }
//     return err;
// }

// bool downstream_i2c_write_read_combined(int bus_idx, uint8_t addr,
//                                         const uint8_t *wdata, size_t wlen,
//                                         uint8_t *rdata, size_t rlen) {
//     pio_i2c_bus_t *b = &buses[bus_idx];
//     if (pio_i2c_write_blocking_nostop(b->pio, b->sm, addr, (uint8_t *)wdata, wlen) < 0)
//         return false;
//     if (pio_i2c_read_blocking(b->pio, b->sm, addr, rdata, rlen) < 0)
//         return false;
//     return true;
// }


#define I2C_TIMEOUT_US 200000

bool downstream_i2c_write_read_combined(int bus_idx, uint8_t addr,
                                        const uint8_t *wdata, size_t wlen,
                                        uint8_t *rdata, size_t rmax,
                                        size_t *rlen)
{
    pio_i2c_bus_t *b = &buses[bus_idx];
    PIO pio = b->pio; uint sm = b->sm;
    bool ok = true;

    // if (!i2c_bus_idle(b->sda_pin, b->scl_pin))
    //     return false;

    pio_i2c_start(pio, sm);

    // ok &= pio_i2c_write_blocking_timeout(pio, sm, (addr << 1) | 0, wdata, wlen, I2C_TIMEOUT_US);
    // if (!ok) goto stop;

    ok &= pio_i2c_read_blocking_timeout(pio, sm, (addr << 1) | 1, rdata, rmax, I2C_TIMEOUT_US);
    if (!ok) goto stop;

stop:
    pio_i2c_stop(pio, sm);
    *rlen = rmax; // or actual bytes read
    return ok;
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
        if (downstream_i2c_write(bus, slave_addr, &reg, 1) &&
            downstream_i2c_read(bus, slave_addr, &val, 1)) {
            printf("  Bus %d: device 0x68 responded, val=0x%02X\n", bus, val);
        } else {
            printf("  Bus %d: no response or read failed\n", bus);
        }
    }
}