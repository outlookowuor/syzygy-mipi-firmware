add_executable(syzygy_mipi)

pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/i2c_multi.pio)
pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/mirror_gpio.pio)
pico_enable_stdio_uart(syzygy_mipi 1)
pico_enable_stdio_usb(syzygy_mipi 1)

target_compile_options(syzygy_mipi PRIVATE -Wall)

target_sources(syzygy_mipi PRIVATE
        syzygy_mipi.c
        syzygy_dna.c
        i2c_muxer.c
        gpio_expander.c
        dna_blob.c
        host_slave_i2cs.c
        clock_programmer.c
        downstream_i2cs.c
        )


target_link_libraries(syzygy_mipi  PRIVATE 
        i2c_pio 
        i2c_slave_multi 
        i2c_slave 
        pico_stdlib 
        hardware_pio 
        hardware_adc 
        hardware_i2c 
        hardware_gpio
        hardware_clocks
        )


pico_add_extra_outputs(syzygy_mipi)