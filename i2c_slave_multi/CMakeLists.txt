add_library(i2c_slave_multi INTERFACE)


target_include_directories(i2c_slave_multi
    INTERFACE
    ./include)

target_sources(i2c_slave_multi
    INTERFACE
    i2c_multi.c
)

target_link_libraries(i2c_slave_multi
    INTERFACE
    hardware_i2c
    hardware_irq
    hardware_pio
)
