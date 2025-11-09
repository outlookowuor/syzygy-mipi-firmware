#include <stdio.h>
#include "pico/stdlib.h"
#include "clock_programmer.h"
#include "hardware/clocks.h"
#include "hardware/clocks.h"
#include "gpio_expander.h"
#include "syzygy_mipi.h"

// --- Protocol Definitions ---
#define MAX_I2C_RX_LEN      7
#define CMD_STOP_CLOCK      0x00
#define CMD_CONFIG_CLOCK    0x01

// RX buffer and index
static uint8_t i2c_rx_buffer[MAX_I2C_RX_LEN];
static volatile uint8_t i2c_rx_index = 0;


// Function Prototypes
static void process_i2c_command();
uint32_t get_auxsrc_frequency(uint8_t auxsrc);
void configure_clock_output(uint8_t gpio, uint8_t auxsrc, uint32_t freq_hz);
void stop_clock_output(uint8_t gpio);

// Master has written data
bool clock_programmer_i2c_receive(uint8_t data){
    if (i2c_rx_index < MAX_I2C_RX_LEN) {
        // Read byte from RX FIFO
        i2c_rx_buffer[i2c_rx_index++] = data;
    } else {
        // Ignore extra bytes if buffer is full
    }

    return true;
}

// Master is requesting data (not used)
bool clock_programmer_i2c_request(uint8_t*  buffer){
    buffer[0] = i2c_rx_buffer[0]; //just send back last command
    // printf("Clock Programmer read request, sending back 0x%02X\n", buffer[0]);
    return true;
}

bool clock_programmer_i2c_restart_request(uint8_t *buffer){
    //same as normal request
    return clock_programmer_i2c_request(buffer);
}

// Master has sent a STOP condition
void clock_programmer_i2c_stop(uint8_t length){
    if (i2c_rx_index > 0) {
        process_i2c_command();
    }
    // Reset buffer index for next command
    i2c_rx_index = 0;
}



//uint8_t clock_gpios[] = {GPIO_23, GPIO_24, GPIO_25};  
// GPIO_21  is strictly for testing - now 23, 24 & 25 on my package
#define GPIO_21 21 
uint8_t clock_gpios[] = {GPIO_21, GPIO_24, GPIO_25};

/**
 * @brief Parses the received I2C buffer and executes the command.
 */
// $ i2cset -y 1 0x17 0x01 0 0 0 0 0 1 i
// $ i2cset -y 1 0x17 0x01 0 0 0 0 2 0 i
// $ i2cset -y 1 0x17 0x01 0 0 0 1 0 0 i
// $ i2cset -y 1 0x17 0x01 0 5 0 1 0 0 i
// $ i2cset -y 1 0x17 0x01 0 3 0 1 0 0 i
// $ i2cset -y 1 0x17 0x00 0 i
// $ i2cset -y 1 0x17 0x01 0 3 0 1 0 0 i
// $ i2cset -y 1 0x17 0x00 0 i
static void process_i2c_command() {
    uint8_t command = i2c_rx_buffer[0];

    switch (command) {
        case CMD_STOP_CLOCK: // 0x00
            if (i2c_rx_index == 2) {
                uint8_t mipi_idx = i2c_rx_buffer[1];
                stop_clock_output(mipi_idx);
            } else {
                printf("Error: STOP_CLOCK invalid length (%d)\n", i2c_rx_index);
            }
            break;

        case CMD_CONFIG_CLOCK: // 0x01
            if (i2c_rx_index == 7) {
                uint8_t mipi_idx = i2c_rx_buffer[1]; // 0,1,2;
                uint8_t auxsrc = i2c_rx_buffer[2];
                
                // Reconstruct the 4-byte frequency (little-endian)
                uint32_t freq_hz = (uint32_t)i2c_rx_buffer[3] |
                                   (uint32_t)(i2c_rx_buffer[4] << 8) |
                                   (uint32_t)(i2c_rx_buffer[5] << 16) |
                                   (uint32_t)(i2c_rx_buffer[6] << 24);
                
                configure_clock_output(mipi_idx, auxsrc, freq_hz);
            } else {
                printf("Error: CONFIG_CLOCK invalid length (%d)\n", i2c_rx_index);
            }
            break;

        default:
            // printf("Error: Unknown command 0x%02X\n", command);
            break;
    }
}

/**
 * @brief Gets the frequency of a clock source based on its auxsrc enum value.
 * @return Frequency in Hz, or 0 if source is invalid.
 */
uint32_t get_auxsrc_frequency(uint8_t auxsrc) {
    switch (auxsrc) {
        case 0: // CLKSRC_PLL_SYS
        case 6: // CLK_SYS
            return clock_get_hz(clk_sys);
        
        case 3: // CLKSRC_PLL_USB
        case 7: // CLK_USB
            return clock_get_hz(clk_usb);

        case 5: // XOSC_CLKSRC
        case 10: // CLK_REF
            return clock_get_hz(clk_ref);

        // case 9: // CLK_RTC
        //     return clock_get_hz(clk_rtc);
        
        // Add others as needed
        default:
            return 0;
    }
}

/**
 * @brief Configures and enables a clock output on a GPIO pin.
 */
void configure_clock_output(uint8_t mipi_idx, uint8_t auxsrc, uint32_t freq_hz) {
    if (!is_valid_mipi(mipi_idx)){
        printf("Error: MIPI %d is not valid\n", mipi_idx);
        return;
    }
    
    uint8_t gpio = clock_gpios[mipi_idx];

    if (freq_hz == 0) {
        printf("Error: Cannot set frequency to 0 Hz.\n");
        return;
    }

    uint32_t src_freq = get_auxsrc_frequency(auxsrc);
    if (src_freq == 0) {
        printf("Error: Invalid clock source 0x%02X\n", auxsrc);
        return;
    }

    // Calculate the divider
    float div = (float)src_freq / (float)freq_hz;
    
    // Divider must be at least 1
    if (div < 1.0f) {
        div = 1.0f;
    }

    printf("Config: GPIO %d, Src 0x%02X (%lu Hz), Freq %lu Hz, Div %.2f\n",
           gpio, auxsrc, src_freq, freq_hz, div);

    // This function handles setting the GPIO function and starting the clock
    clock_gpio_init(gpio, auxsrc, div);
}

/**
 * @brief Stops the clock output on a GPIO pin.
 */
void stop_clock_output(uint8_t mipi_idx) {
    if (!is_valid_mipi(mipi_idx)) {
        printf("Error: MIPI %d is not valid.\n", mipi_idx);
        return;
    }

    printf("Stop: MIPI %d clock\n", mipi_idx);

    uint8_t gpio = clock_gpios[mipi_idx];

    // Stop the corresponding GPOUT channel to save power
    if (gpio == 23) clock_stop(clk_gpout1);
    if (gpio == 24) clock_stop(clk_gpout2);
    if (gpio == 25) clock_stop(clk_gpout3);

    // Set the pin back to a standard SIO (Software I/O) function
    // and disable it.
    gpio_init(gpio);
    gpio_set_function(gpio, GPIO_FUNC_SIO);
    gpio_disable_pulls(gpio);
    gpio_set_input_enabled(gpio, false);
}