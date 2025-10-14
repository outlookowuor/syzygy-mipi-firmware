#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "syzygy_dna.h"
#include "i2c_fifo.h"
#include "i2c_slave.h"
#include "dna_blob.h"



#define GA_VALID_BUFFER_MV 50  // allow for +/-50mV from each GA value to account for noise, etc.


uint8_t adc_to_channel(uint16_t adc_millivolts);
uint16_t read_adc_millivolts(int gpio_pin);

#define RGA_ADC_PIN 26 // GPIO 26 is ADC0  - GPIO47_ADC7 - RP2350
#define P3V3_ADC_PIN 27 // GPIO 27 is ADC1  - GPIO44_ADC4  - RP2350



static int syzygy_channel_address = -1;

int retrieve_channel_address() {
    adc_init();
    uint16_t rga_millivolts = read_adc_millivolts(RGA_ADC_PIN);
    uint16_t p3v3_millivolts = read_adc_millivolts(P3V3_ADC_PIN);

    if (rga_millivolts > 0 && p3v3_millivolts > 0) {
        // Normalize RGA reading to P3V3
        //P3V3 should be a precise 500mV 
        //We scale RGA to ratio of P3V3
        rga_millivolts = (rga_millivolts * 500) / p3v3_millivolts;
        syzygy_channel_address = adc_to_channel(rga_millivolts);
        return syzygy_channel_address;
    }
    else 
        printf("Error reading ADC values\n");
    return -1; 
}


uint16_t read_adc_millivolts(int gpio_pin) {
    adc_gpio_init(gpio_pin); // Initialize the GPIO pin for ADC
    adc_select_input(gpio_pin - 26); // Select the ADC input (0 for GPIO26, 1 for GPIO27, etc.)

    // 12-bit conversion, assume max value == ADC_VREF == 3.3 V
    // perhaps should read multiple times and average ? 
    const float conversion_factor = 3.3f / (1 << 12);
    uint16_t result = adc_read();
    printf("Raw value: 0x%03x, voltage: %f V\n", result, result * conversion_factor);
    
    sleep_ms(10); // Allow some time for the ADC to stabilize
    return (uint16_t)(result * conversion_factor * 1000); // Return in millivolts
}


// Voltages in mV of each geographical address resistance measurement
// Defined in SYZYGY Specification v1.0
// This assumes a 3.3V reference and a 10k pull-up on the peripheral.
const uint16_t adc_val_ga[] = {3147, 2944, 2740, 2538, 2341, 2135, 1926, 1734,
                               1535, 1341, 1137, 933, 738, 541, 342, 153};

// Geographical address corresponding to each potential resistor reading
// Defined in the SYZYGY specification v1.0
const uint8_t geo_addr[16] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
                              0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F};

uint8_t adc_to_channel(uint16_t adc_millivolts) {
	uint8_t i = 0;

	for (i = 0; i < 16; i++){
		if ((adc_millivolts > (adc_val_ga[i] - GA_VALID_BUFFER_MV))
		 && (adc_millivolts < (adc_val_ga[i] + GA_VALID_BUFFER_MV))) {
			return geo_addr[i];
		}
	}

    return 0xFF; // Invalid channel
}


#define I2C_DNA_SDA_PIN 0
#define I2C_DNA_SCL_PIN 1
#define I2C_BAUDRATE 100000

static struct
{
    uint8_t mem[256];
    uint8_t mem_address;
    uint16_t sub_address_write;
    uint16_t sub_address_read;
    uint8_t sub_address_bytes;
} context = {
    .mem = {0},
    .mem_address = 0,
    .sub_address_write = 0,
    .sub_address_read = 0,
    .sub_address_bytes = 0
};


#define FW_DNA_BYTES 1024 // Fictitious size of the DNA blob
const uint8_t syzygy_reg[] = {0x01, 0x01, 0x01, 0x00, FW_DNA_BYTES >> 8, FW_DNA_BYTES & 0xFF};
#define SYZYGY_REG_SIZE 6 // The syzygy_reg setup contains 6 bytes
#define DNA_EEPROM_START 0x8000

static void write_reserved(uint16_t addr, uint8_t value) {
    // Placeholder for writing to reserved memory
    // In a real implementation, this would write to specific hardware registers
    printf("Write Reserved: Addr=0x%04X, Value=0x%02X\n", addr, value);

}

void write_pmcu_registers(uint16_t addr, uint8_t value) {
    if (addr < SYZYGY_REG_SIZE) {
        printf("Write PMCU: Addr=0x%04X, Value=0x%02X\n", addr, value);
    } else {
        printf("Attempt to Write Invalid PMCU: Addr=0x%04X\n", addr);
    }
}

static void write_dna_eeprom(uint16_t addr, uint8_t value) {
    if (addr < DNA_EEPROM_START || addr >= DNA_EEPROM_START + dna_blob_len) {
        printf("Write DNA : Addr=0x%04X out of range\n", addr);
        return; // out of range
    }
    // We are not doing eeprom writes, so just log it for now
    printf("Write DNA : Addr=0x%04X, Valuoverwrite the blob in memorye=0x%02X\n", addr, value);
}

static uint8_t read_reserved(uint16_t addr) {
    // Placeholder for writing to reserved memory
    printf("Read Reserved: Addr=0x%04X\n", addr);
    return 0xFF; // Dummy value
}

static uint8_t read_dna_eeprom(uint16_t addr) {
    // We are not doing eeprom writes, so just read from the blob
    if (addr < DNA_EEPROM_START || addr >= DNA_EEPROM_START + dna_blob_len) {
        printf("Read DNA : Addr=0x%04X out of range\n", addr);
        return 0xFF; // out of range
    }
    return dna_blob[addr - DNA_EEPROM_START];
}

static uint8_t read_pmcu_registers(uint16_t addr) {
    if (addr < SYZYGY_REG_SIZE) {
		return syzygy_reg[addr];
	}
    printf("Attempt to Read Invalid PMCU: Addr=0x%04X\n", addr);
	return 0xFF;
}   

// Attempt to translate AVR code in syzygy reference firmware
// into piconese while referencing the syzygy DNA specification
// WIP
static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    switch (event) {
    case I2C_SLAVE_RECEIVE: // master has written some data
        if (context.sub_address_bytes < 2) {
            // writes always start with the sub address
            uint8_t in_byte = i2c_read_byte(i2c);

            if (context.sub_address_bytes == 0) {  // high byte first
                context.sub_address_write = in_byte << 8;
            } else {
                context.sub_address_write |= in_byte; // low byte second

                //We don't know whether we're going to read or write yet, 
                //so set read address to write address
                context.sub_address_read = context.sub_address_write; 
            }
            context.sub_address_bytes++;
        } else {
            // subsequent writes are data to be stored in memory
            uint8_t in_byte = i2c_read_byte(i2c);
            if (context.sub_address_write >= 0x9000) {
                write_reserved(context.sub_address_write, in_byte);
            } else if (context.sub_address_write >= 0x8000) {
                write_dna_eeprom(context.sub_address_write, in_byte);
            } else {
                write_pmcu_registers(context.sub_address_write, in_byte);
            }
            context.sub_address_write++;
        }
        break;
    case I2C_SLAVE_REQUEST: // master is requesting data
        // load from memory
        uint8_t out_byte = 0;
        uint16_t sub_addr_read = context.sub_address_read;

        if (sub_addr_read >= 0x9000) {
            out_byte = read_reserved(sub_addr_read);
		} else if (sub_addr_read >= 0x8000 && sub_addr_read < 0x8000 + FW_DNA_BYTES) {
			out_byte = read_dna_eeprom(sub_addr_read);
		} else {
            out_byte = read_pmcu_registers(sub_addr_read);
		}
        i2c_write_byte(i2c, out_byte);
        //advance the read address for next time
        context.sub_address_read++;
        break;
    case I2C_SLAVE_FINISH: // master has signalled Stop / Restart
        context.sub_address_bytes = 0;
        context.mem_address = 0;
        break;
    default:
        break;
    }
}

#define DNA_I2C_SDA_PIN 0
#define DNA_I2C_SCL_PIN 1
#define I2C_BAUDRATE 100000

static void init_dna_i2c(uint8_t channel_address) {
    gpio_init(DNA_I2C_SDA_PIN);
    gpio_set_function(DNA_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(DNA_I2C_SDA_PIN);

    gpio_init(DNA_I2C_SCL_PIN);
    gpio_set_function(DNA_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(DNA_I2C_SCL_PIN);

    i2c_init(i2c0, I2C_BAUDRATE);
    // configure I2C0 for slave mode
    
    i2c_slave_init(i2c0, channel_address, &i2c_slave_handler);
}

int setup_syzygy_dna() {
    
    syzygy_channel_address = retrieve_channel_address(); 
    if (syzygy_channel_address != -1) {
        printf("Syzygy Channel Address: %d\n", syzygy_channel_address);
    } else {
        printf("Failed to retrieve Syzygy Channel Address\n");
    }

    init_dna_i2c(syzygy_channel_address);

    return 0;
}
