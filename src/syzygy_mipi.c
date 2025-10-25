#include <stdio.h>

#include "pico/stdlib.h"
#include "syzygy_dna.h"
#include "i2c_muxer.h"
#include "gpio_expander.h"
#include "host_slave_i2cs.h"
#include "syzygy_mipi.h"

#include "hardware/uart.h"
#include "hardware/gpio.h"


#define BAUD_RATE 115200
#define UART_ID uart0
#define UART_TX_PIN 4
#define UART_RX_PIN 5

int main() {
    stdio_init_all();

        // Set up our UART
    uart_init(UART_ID, BAUD_RATE);
    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);


    printf("SyZyGy MiPi Adaptor Firmware\n");

    setup_host_i2cs();

    setup_syzygy_dna();  //slave on i2c0
    printf("SyZyGy DNA is Ready\n");

    setup_gpio_expander(); // 3x GPIOs connected to MIPI devices
    printf("GPIO Expander is Ready\n");

    setup_i2c_muxer();  // 3 x I2C controllers connected to MIPI devices
    printf("i2c Multiplexer is Ready\n");


    while (1) {
        tight_loop_contents();
    }
        
    return 0;
}

bool is_valid_mipi(uint8_t mipi_idx){ return mipi_idx < NUM_MIPIS; }


