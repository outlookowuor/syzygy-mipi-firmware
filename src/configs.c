#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "syzygy_mipi.h"
#include "syzygy_dna.h"

#include "configs.h"

#define CONFIG_MAGIC 0x524E1D1 //SYZYGY_MIPI ha ha
#define CONFIG_FLASH_OFFSET  (PICO_FLASH_SIZE_BYTES - 4096)


bool config_modified = false;

syzygy_mipi_config_t config = {0};


static void save_config(void) {
    if (config_modified == false) {
        printf("No changes to save.\n");
        return; //no changes, save flash writes
    }

    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(CONFIG_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(CONFIG_FLASH_OFFSET, (uint8_t *)&config, sizeof(config));
    restore_interrupts(ints);
    config_modified = false;
}

static bool load_config(void) {
    const syzygy_mipi_config_t *stored = (const syzygy_mipi_config_t *)(XIP_BASE + CONFIG_FLASH_OFFSET);
    if (stored->magic != CONFIG_MAGIC) return false;
    config = *stored;
    return true;
}


syzygy_mipi_config_t config;


/**
    * Initialize configuration: load from flash or set defaults
    */

void init_configs(){
    if (!load_config()){
        //use defaults
        config.muxer_addr = DEFAULT_MUXER_ADDRESS;
        config.gpio_expander_addr = DEFAULT_GPIO_EXPANDER_ADDRESS;
        config.clock_programmer_addr = DEFAULT_CLOCK_PROGRAMMER_ADDRESS;
        config.bridge_addr = DEFAULT_BRIDGE_ADDRESS;
        config.default_mipi_idx = 0;
        // save_config();
    }
}


#define MUXER_PROMPT "I2C Muxer"
#define GPIO_EXPANDER_PROMPT "GPIO Expander"
#define CLOCK_PROGRAMMER_PROMPT "Clock Programmer"
#define BRIDGE_PROMPT "Bridge MIPI"

#define MENU_MUXER '2'
#define MENU_BRIDGE '3'
#define MENU_GPIO_EXPANDER '4'
#define MENU_CLOCK_PROGRAMMER '5'
#define MENU_SYZGY_DNA '6'
#define MENU_SAVE '7'



void show_menu(void) {
    printf("\n=== SyZyGy-MIPI Adapter Config Menu ===\n");
    printf("1. Show current I2C addresses\n");
    printf("%c. Set address for i2c Multiplexer\n", MENU_MUXER);
    printf("%c. Set address for selected MIPI device\n", MENU_BRIDGE);
    printf("%c. Set address for gpio expander\n", MENU_GPIO_EXPANDER);
    printf("%c. Set address for clock generator\n", MENU_CLOCK_PROGRAMMER);
    printf("%c. Show SYZYGY DNA\n", MENU_SYZGY_DNA);
    printf("%c. Save address settings\n", MENU_SAVE);
    printf("q. Quit\n");
    printf("> ");
}

void show_addresses(void) {
    printf("\nCurrent I2C Addresses:\n");

    printf("  Muxer: 0x%02X\n", config.muxer_addr);
    printf("  GPIO Expander: 0x%02X\n", config.gpio_expander_addr);
    printf("  Clock Programmer: 0x%02X\n", config.clock_programmer_addr);
    printf("  Bridge MIPI: 0x%02X\n", config.bridge_addr);
}

void set_address(char menu_option){ 
    uint8_t new_addr, *p_addr;
    char *prompt;

    switch (menu_option) {
        case MENU_MUXER:
            prompt = MUXER_PROMPT;
            p_addr = &config.muxer_addr;
            break;
        case MENU_GPIO_EXPANDER:
            prompt = GPIO_EXPANDER_PROMPT;
            p_addr = &config.gpio_expander_addr;
            break;
        case MENU_CLOCK_PROGRAMMER:
            prompt = CLOCK_PROGRAMMER_PROMPT;
            p_addr = &config.clock_programmer_addr;
            break;
        case MENU_BRIDGE:
            prompt = BRIDGE_PROMPT;
            p_addr = &config.bridge_addr;
            break;
        case MENU_SYZGY_DNA:
            printf("SYZYGY DNA:\n");
            print_syzygy_dna();
            return;
        default:
            printf("Invalid address!\n");
            return;
    }

    printf("Enter new I2C address (hex, e.g. 0x54): 0x");
    scanf("%x", &new_addr);
    if (new_addr > 0x7F) {
        printf("Invalid I2C 7-bit address!\n");
        return;
    }

    if (*p_addr == new_addr) {
        printf("%s address is already 0x%02X\n", prompt, new_addr);
        return;
    }

    config_modified = true;
    *p_addr = new_addr;
    printf("Updated %s address to 0x%02X\n", prompt, new_addr);
}


void config_menu_loop(){
    sleep_ms(2000);  // Give terminal time to attach
    while (1) {
        show_menu();

        char choice = getchar();
        printf("%c\n", choice);

        switch (choice) {
            case '1': show_addresses(); break;
            case MENU_MUXER:
            case MENU_BRIDGE: 
            case MENU_GPIO_EXPANDER: 
            case MENU_CLOCK_PROGRAMMER:  
                set_address(choice); 
                break;
            case MENU_SYZGY_DNA:
                print_syzygy_dna();
                break;
            case MENU_SAVE: 
                printf("Saving (not implemented)... rebooting.\n");
                save_config();
                break;
            case 'q': 
            case 'Q':
                printf("Exiting config menu.\n");
                break;
            default: 
                printf("Unknown option.\n");
                break;
        }
    }
}