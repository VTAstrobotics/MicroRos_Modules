cmake_minimum_required(VERSION 3.12)

set(PICO_SDK_FETCH_FROM_GIT on)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

include(pico_sdk_import.cmake)

project(imu_pico C CXX ASM)

pico_sdk_init()

link_directories(libmicroros)
add_executable(imu_pico
    pico_uart_transport.c
    pico_uart_transport.h
    imu_pico.c
)
target_link_libraries(imu_pico
    pico_stdlib
    microros
    hardware_i2c
    hardware_gpio
)

target_include_directories(imu_pico PUBLIC
    libmicroros/include
    ${CMAKE_CURRENT_SOURCE_DIR}
)

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Configure Pico
pico_enable_stdio_usb(imu_pico 1)
pico_enable_stdio_uart(imu_pico 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate UF2
pico_add_extra_outputs(imu_pico)


