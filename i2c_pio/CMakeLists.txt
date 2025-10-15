add_library(i2c_pio INTERFACE)

target_include_directories(i2c_pio
    INTERFACE
    ./include)

target_sources(i2c_pio
    INTERFACE
    pio_i2c.c
)

target_link_libraries(i2c_pio
    INTERFACE
    hardware_i2c
    hardware_irq
)

