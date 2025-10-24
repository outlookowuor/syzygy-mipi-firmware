#include <stdio.h>

#include "pico/stdlib.h"
#include "syzygy_dna.h"
#include "i2c_muxer.h"
#include "gpio_expander.h"
#include "host_slave_i2cs.h"
#include "syzygy_mipi.h"


int main() {
    stdio_init_all();
    printf("SyZyGy MiPi Adaptor Firmware\n");

    setup_syzygy_dna();  //slave on i2c0
    printf("SyZyGy DNA is Ready\n");

    setup_gpio_expander(); // 3x GPIOs connected to MIPI devices
    printf("GPIO Expander is Ready\n");

    setup_i2c_muxer();  // 3 x I2C controllers connected to MIPI devices
    printf("i2c Multiplexer is Ready\n");

    setup_host_i2cs();

    while (1) {
        tight_loop_contents();
    }
        
    return 0;
}

bool is_valid_mipi(uint8_t mipi_idx){ return mipi_idx < NUM_MIPIS; }


