#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "syzygy_mipi.h"
#include "gpio_expander.h"
#include "mirror_gpio.pio.h"
/**
 * GPIO 'expander' is a programmer for the GPIOs 
 * connected to MIPI devices. More
 * 
 * Each MIPI device has 2 GPIOs. We'll call them GPIO_0 and GPIO_1
 * 
 * By default
 * GPIO_0 is input/output, interrupt programmable
 * GPIO_1 is clock_programmable (clock_generator)
 *
 */

static uint32_t mipi_gpios[3]= { GPIO_18, GPIO_19, GPIO_20 };

static void init_pio_mirroring();
static void mirror_gpio(PIO pio, uint sm, uint offset,
                 uint input_pin, uint output_pin,
                 const pio_program_t *program);


static void handle_command(uint8_t *cmd);


int setup_gpio_expander() {
    //nothing for now
    init_pio_mirroring();
    return 0;
}


// Simple 3-byte command buffer - for
uint8_t cmd_buf[3];
// uint8_t code = cmd[0];
// uint8_t gpio = cmd[1];
// uint8_t val  = cmd[2];

static uint8_t buf_index = 0;
//master write byte 'data' to us
void gpio_expander_i2c_write_byte(uint8_t data){
    if (buf_index < 3)
        cmd_buf[buf_index++] = data;
    if (buf_index == 3) {
        handle_command(cmd_buf);
        buf_index = 0;
    }
}
void gpio_expander_i2c_read_byte(uint8_t *buffer){
    buffer[0] = cmd_buf[1];
}

void gpio_expander_i2c_stop(uint8_t length){
    buf_index = 0;
}

// $ i2cset -y 1 0x50 0x01 0 1 i
// $ i2cset -y 1 0x50 0x01 1 1 i
// $ i2cset -y 1 0x50 0x03 0 1 i
// $ i2cset -y 1 0x50 0x03 0 1 i
// $ i2cset -y 1 0x17 0x00 0 i

static PIO pio = pio2;  // 3rd PIO for the expander
static uint pio_sm  = 0;
static uint program_offset;

// Command codes
#define CMD_SET_OUTPUT  0x01
#define CMD_READ_INPUT  0x02
#define CMD_SET_PULL    0x03

//simple - for testing purposes
void handle_command(uint8_t *cmd) {
    uint8_t code = cmd[0];
    uint8_t mipi = cmd[1];
    uint8_t val  = cmd[2];

    if (!is_valid_mipi(mipi)){
        printf("Invalid MIPI: %d\n", mipi);
        return;
    }
    // Make sure GPIO is initialized
    uint32_t gpio = mipi_gpios[mipi];
    gpio_init(gpio);

    uint32_t host_gpio = GPIO_HOST;
    gpio_init(host_gpio);

    switch (code) {
        case CMD_SET_OUTPUT:
            gpio_set_dir(gpio, GPIO_OUT);
            gpio_put(gpio, val);

            mirror_gpio(pio, pio_sm, program_offset, host_gpio, 
                gpio, &mirror_gpio_program);

            break;

        case CMD_READ_INPUT:
            gpio_set_dir(gpio, GPIO_IN);
            {
                bool level = gpio_get(gpio);
                printf("GPIO %d read = %d\n", gpio, level);
            }

            mirror_gpio(pio, pio_sm, program_offset, gpio, 
                host_gpio, &mirror_gpio_program);

            break;

        case CMD_SET_PULL:
            gpio_set_dir(gpio, GPIO_IN);
            gpio_disable_pulls(gpio);
            if (val == 1) gpio_pull_up(gpio);
            else if (val == 2) gpio_pull_down(gpio);

            mirror_gpio(pio, pio_sm, program_offset, gpio, 
                host_gpio, &mirror_gpio_program);

            break;

        default:
            printf("Unknown command 0x%02X\n", code);
            break;
    }
}

void init_pio_mirroring(){
    pio_sm = pio_claim_unused_sm(pio, true);
    program_offset = pio_add_program(pio, &mirror_gpio_program);
}

void mirror_gpio(PIO pio, uint sm, uint offset,
                 uint input_pin, uint output_pin,
                 const pio_program_t *program)
{
    pio_sm_set_enabled(pio, sm, false);

    // Keep output aligned with input before starting
    gpio_put(output_pin, gpio_get(input_pin));

    pio_sm_config c = mirror_gpio_program_get_default_config(offset);
    sm_config_set_in_pins(&c, input_pin);
    sm_config_set_set_pins(&c, output_pin, 1);
    sm_config_set_out_pins(&c, output_pin, 1);

    pio_gpio_init(pio, output_pin);
    pio_sm_set_consecutive_pindirs(pio, sm, output_pin, 1, true);

    pio_gpio_init(pio, input_pin);
    gpio_pull_down(input_pin);
    pio_sm_set_consecutive_pindirs(pio, sm, input_pin, 1, false);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}





/**
 * data byte is interpreted as follows:
 * 
 * [1:0]: Select "which GPIO"
 * [3:2]: indicates whether to set it High/Low or just float
 * [4:4]: input / output
 * [6:5]: pull-up, pull-down, floating
 * [7:7]: 
//  */
// static uint8_t gpio_setting;    

// void do_gpio_setting(uint8_t bits){

//     //Bits[1:0] indicate which MIPI: 0, 1, 2
//     uint32_t gpio = mipi_gpios[bits & 0x3];
//     gpio_init(gpio);

//     uint32_t host_gpio = GPIO_HOST;
//     gpio_init(host_gpio);


//     //Bits [3:2]

//     // Bit [4:4]
//     bool is_input = (bits & 0x8) > 0;

//     if (!is_input) { // output pin
//         gpio_set_dir(gpio, GPIO_OUT);
//         gpio_set_dir(host_gpio, GPIO_IN);
            
//         uint set_value = 0;
//         uint set_now = (bits >> 2) & 0x3;
//         if (set_now == 0x01) // 0b01 -> Set to 1
//             set_value = 1;
//         else if (set_now == 0x2)
//             set_value = 0;   // 0b10  -> Set to 0
//         else {
//             set_now = 0;  // 0b11 -> taken as an edge interrupt
//         }
//         if (set_now>0){
//             gpio_put(gpio, set_value);
//         }

//         mirror_gpio(pio, pio_sm, program_offset, host_gpio, 
//             gpio, &mirror_gpio_program);
//     }

//     if (is_input) {
//         gpio_set_dir(gpio, GPIO_IN);
//         gpio_set_dir(host_gpio, GPIO_OUT);
//         //Bit[7:7] pull up / pull down  
//         if ((bits & 0x8) > 0){
//             gpio_pull_up(gpio);
//         }
//         else{
//             gpio_pull_down(gpio);
//         }
//         mirror_gpio(pio, pio_sm, program_offset, gpio, 
//             host_gpio, &mirror_gpio_program);
//     }
// }