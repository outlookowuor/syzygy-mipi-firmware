#ifndef CONFIG_MENU_H
#define CONFIG_MENU_H   

typedef struct {
    uint32_t magic;
    uint8_t muxer_addr;
    uint8_t gpio_expander_addr;
    uint8_t clock_programmer_addr;
    uint8_t bridge_addr;
    uint8_t default_mipi_idx;
} syzygy_mipi_config_t;

extern syzygy_mipi_config_t config;

void config_menu_loop();
void init_configs();

#endif // CONFIG_MENU_H