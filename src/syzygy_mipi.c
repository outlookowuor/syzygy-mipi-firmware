#include <stdio.h>

#include "pico/stdlib.h"
#include "syzygy_dna.h"
#include "i2c_muxer.h"
#include "gpio_expander.h"
#include "host_slave_i2cs.h"
#include "syzygy_mipi.h"

#include "hardware/uart.h"
#include "configs.h"

#define BAUD_RATE 115200
#define UART_ID uart0
#define UART_TX_PIN 16
#define UART_RX_PIN 17


int main() {
    // stdio_init_all();
    init_configs();

    stdio_uart_init_full(UART_ID, BAUD_RATE,  UART_TX_PIN, UART_RX_PIN); 

    for (int i = 0; i < 3; i++) {
        printf("\nTesing UART\r\n");
        sleep_ms(1000);
    }
    printf("SyZyGy MiPi Adaptor Firmware\n");

    setup_host_i2cs();

    setup_syzygy_dna();  //slave on i2c0
    printf("SyZyGy DNA is Ready\n");

    setup_gpio_expander(); // 3x GPIOs connected to MIPI devices
    printf("GPIO Expander is Ready\n");

    setup_i2c_muxer();  // 3 x I2C controllers connected to MIPI devices
    printf("i2c Multiplexer is now Ready\n");


    while (1) {
        // tight_loop_contents();
        config_menu_loop();

    }
        
    return 0;
}

bool is_valid_mipi(uint8_t mipi_idx){ return mipi_idx < NUM_MIPIS; }



