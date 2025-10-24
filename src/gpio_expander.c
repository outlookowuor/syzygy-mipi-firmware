#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "syzygy_mipi.h"
#include "gpio_expander.h"
/**
 * GPIO 'expander' is botha selector and programmer for the GPIOs 
 * connected to MIPI devices.
 * 
 * Each MIPI device has 2 GPIOs. We'll call them GPIO_0 and GPIO_1
 * 
 * By default
 * GPIO_0 is input/output, interrupt programmable
 * GPIO_1 is clock_programmable (clock_generator)
 *
 */

static uint8_t gpio_setting;    
void gpio_irq_callback(uint gpio, uint32_t events);



static void do_gpio_setting(uint8_t bits);

int setup_gpio_expander() {
    //nothing for now

    return 0;
}


void gpio_expander_i2c_write_byte(uint8_t data){
    do_gpio_setting(data);
}
void gpio_expander_i2c_read_byte(uint8_t *buffer){
    buffer[0] = gpio_setting;
}

void gpio_expander_i2c_stop(uint8_t length){

}


/**
 * data byte is interpreted as follows:
 * 
 * [1:0]: Select "which GPIO"
 * [3:2]: indicates which GPIO to set
 * [4:4]: input / output
 * [6:5]: trigger interrupt enabled edge
 * [7:7]: swap the GPIOs? 
 */


 #define GPIO_18 18
 #define GPIO_19 19
 #define GPIO_20 20
 #define GPIO_23 23
 #define GPIO_24 24
 #define GPIO_26 26

uint32_t mipi_gpios[3][2] = {
    { GPIO_18, GPIO_23 },
    { GPIO_19, GPIO_24 },
    { GPIO_20, GPIO_26 }
};

void do_gpio_setting(uint8_t bits){

    //Bits[1:0] indicate which MIPI: 0, 1, 2
    uint32_t gpio = mipi_gpios[bits & 0x3][0];
    gpio_init(gpio);

    //Bits [3:2]
    uint set_value = 0;
    uint level_interrupt = 0;

    // Bit [4:4]
    bool is_input = (bits & 0x8 > 0);

    if (!is_input) { // output pin
        gpio_set_dir(gpio, GPIO_OUT);
            
        uint set_now = (bits >> 2) & 0x3;
        if (set_now == 0x01) // 0b01 -> Set to 1
            set_value = 1;
        else if (set_now = 0x2)
            set_value = 0;   // 0b10  -> Set to 0
        else {
            set_now = 0;  // 0b11 -> taken as an edge interrupt
        }
        if (set_now>0){
            gpio_put(gpio, set_value);
        }
    }

    if (is_input) {
        gpio_set_dir(gpio, GPIO_IN);
        //Bit[7:7] pull up / pull down  
        if ((bits & 0x8) > 0)
            gpio_pull_up(gpio);
        else
            gpio_pull_down(gpio);
        

        uint32_t event_mask = GPIO_IRQ_EDGE_RISE; 

        // Bits [6:5] - input can be interrupted    
        int interrupt_type = (bits >> 4) & 0x3;
        int level_interrupt = 0;
        if ((bits >> 2) & 0x3 == 0x3)
            level_interrupt = 1;

        if (!is_input)
            interrupt_type = 0; //NONE

        else if (interrupt_type & 0x3){
            //both edges
        }
        else if (interrupt_type & 0x2) {
            if (level_interrupt){
                //on Low
            }
            else{
            //falling edges
            }
        }
        else if (interrupt_type & 0x1) {

            if (level_interrupt){
                //on High
            }
            else {
                //rising edge
            }
        }
        if (interrupt_type>0){
            gpio_set_irq_enabled_with_callback( gpio, event_mask,      
                true, &gpio_irq_callback);
        }
    }




}

void interrupt_host(uint32_t events){
    printf("Host was interrupted with: %d\n",events);
}


void gpio_irq_callback(uint gpio, uint32_t events) {
    for (int i=0; i<NUM_MIPIS; i++){
        if (gpio == mipi_gpios[i][0]){
            if (i = selected_mipi_device){
                interrupt_host(events);
            }
        }
    }
}