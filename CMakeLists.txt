# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(syzygy_mipi C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()


include_directories(
    ${PROJECT_SOURCE_DIR}/i2c_slave/include
    ${PROJECT_SOURCE_DIR}/i2c_pio/include
    ${PROJECT_SOURCE_DIR}/i2c_slave_multi/include
)

add_subdirectory(i2c_slave_multi)
add_subdirectory(i2c_slave)
add_subdirectory(i2c_pio)
add_subdirectory(src)


set (PICO_DEFAULT_UART 1)
set(PICO_DEFAULT_UART_TX 4)
set(PICO_DEFAULT_UART_RX 5)

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 1)
