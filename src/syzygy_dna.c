#include <stdio.h>
#include <string.h>
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

// Translate AVR code in syzygy reference firmware
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


/// Computes the CRC-16/CCITT checksum using parallel computation without tables.
/// https://en.wikipedia.org/wiki/Computation_of_cyclic_redundancy_checks
///
/// Polynomial: 0x1021 (x^16 + x^12 + x^5 + x^0)
/// Initialization: 0xFFFF
/// Data “shifted” MSB first
///
/// \returns Computed 16-bit CRC.

static uint16_t crc16_usb(const uint8_t *data, size_t len) {
    uint16_t crc = 0xffff;
    short x;


    while (len--) {
        x = (crc>>8) ^ *data++;
        x ^= x>>4;
        crc = (crc<<8) ^ (x<<12) ^ (x<<5) ^ (x);
        crc &= 0xffff;
    }
    return(crc);
}


#pragma pack(push, 1)  // ensure no padding

typedef struct {
    uint16_t dna_full_data_length;
    uint16_t dna_header_length;

    uint8_t  dna_major_version;
    uint8_t  dna_minor_version;
    uint8_t  req_dna_major_version;
    uint8_t  req_dna_minor_version;

    uint16_t max_5v_load_mA;
    uint16_t max_3v3_load_mA;
    uint16_t max_vio_load_mA;

    uint16_t attribute_flags;

    uint16_t min_vio_10mV;
    uint16_t max_vio_10mV;

    uint16_t min_vio_extra[4];   // reserved fields (often 0)
    uint16_t max_vio_extra[4];

    uint8_t  manufacturer_name_length;
    uint8_t  product_name_length;
    uint8_t  product_model_length;
    uint8_t  product_version_length;
    uint8_t  serial_number_length;

    uint8_t  reserved;

    uint8_t  crc_msb;
    uint8_t  crc_lsb;
} syzygy_dna_header_t;

#pragma pack(pop)

void print_syzygy_dna() {
    const uint8_t *data = dna_blob;
    size_t len = dna_blob_len;

    if (len < sizeof(syzygy_dna_header_t)) {
        fprintf(stderr, "Error: buffer too small (%zu bytes)\n", len);
        return;
    }

    const syzygy_dna_header_t *hdr = (const syzygy_dna_header_t *)data;
    const uint8_t *p = data + sizeof(syzygy_dna_header_t);

    printf("=== SYZYGY DNA Record ===\n");
    printf("Full data length:       %u bytes\n", hdr->dna_full_data_length);
    printf("Header length:          %u bytes\n", hdr->dna_header_length);
    printf("DNA version:            %u.%u\n", hdr->dna_major_version, hdr->dna_minor_version);
    printf("Required version:       %u.%u\n", hdr->req_dna_major_version, hdr->req_dna_minor_version);

    printf("\n--- Electrical specs ---\n");
    printf("Max 5V load:            %u mA\n", hdr->max_5v_load_mA);
    printf("Max 3.3V load:          %u mA\n", hdr->max_3v3_load_mA);
    printf("Max VIO load:           %u mA\n", hdr->max_vio_load_mA);
    printf("Attribute flags:        0x%04X\n", hdr->attribute_flags);
    printf("VIO range:              %.2f – %.2f V\n",
           hdr->min_vio_10mV / 100.0, hdr->max_vio_10mV / 100.0);

    printf("\n--- String lengths ---\n");
    printf("Manufacturer:           %u\n", hdr->manufacturer_name_length);
    printf("Product name:           %u\n", hdr->product_name_length);
    printf("Product model:          %u\n", hdr->product_model_length);
    printf("Product version:        %u\n", hdr->product_version_length);
    printf("Serial number:          %u\n", hdr->serial_number_length);

    printf("\n--- CRC ---\n");
    uint16_t stored_crc = ((uint16_t)hdr->crc_msb << 8) | hdr->crc_lsb;
    uint16_t computed_crc = crc16_usb(data, len - 2); // exclude CRC bytes
    printf("Stored CRC:             0x%04X\n", stored_crc);
    printf("Computed CRC:           0x%04X  (%s)\n",
           computed_crc, (stored_crc == computed_crc) ? "OK" : "FAIL");

    // Now extract variable-length strings
    printf("\n--- Strings ---\n");
    char buf[256];

    #define COPY_STRING(label, length_field) \
        memset(buf, 0, sizeof(buf)); \
        if ((p + hdr->length_field) > (data + len)) { \
            printf("%-22s <invalid length>\n", label); \
            return; \
        } \
        memcpy(buf, p, hdr->length_field); \
        printf("%-22s %s\n", label, buf); \
        p += hdr->length_field;

    COPY_STRING("Manufacturer:", manufacturer_name_length);
    COPY_STRING("Product name:", product_name_length);
    COPY_STRING("Product model:", product_model_length);
    COPY_STRING("Product version:", product_version_length);
    COPY_STRING("Serial number:", serial_number_length);

    #undef COPY_STRING

    printf("=========================\n");
}