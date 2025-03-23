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
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(pico_cnc_hmi C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

cmake_minimum_required(VERSION 3.13)

add_executable(pico_cnc_hmi)

target_sources(pico_cnc_hmi PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/main.cc
        ${CMAKE_CURRENT_LIST_DIR}/encoder.cc
        ${CMAKE_CURRENT_LIST_DIR}/spi.cc
        ${CMAKE_CURRENT_LIST_DIR}/oled_sh1122.cc
        ${CMAKE_CURRENT_LIST_DIR}/i2c.cc
        ${CMAKE_CURRENT_LIST_DIR}/tca8418.cc
        ${CMAKE_CURRENT_LIST_DIR}/tlc59116.cc
        ${CMAKE_CURRENT_LIST_DIR}/usb_descriptors.c
        )

# Make sure TinyUSB can find tusb_config.h
target_include_directories(pico_cnc_hmi PUBLIC
        ${CMAKE_CURRENT_LIST_DIR})

# LVGL submodule
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/lv_conf.h DESTINATION ${CMAKE_BINARY_DIR}/lvgl)
add_subdirectory(lvgl)

# In addition to pico_stdlib required for common PicoSDK functionality, add dependency on tinyusb_device
# for TinyUSB device support and tinyusb_board for the additional board support library used by the example
target_link_libraries(pico_cnc_hmi PUBLIC pico_stdlib pico_stdio pico_unique_id tinyusb_device tinyusb_board hardware_pio hardware_i2c hardware_spi lvgl)

# stdio serial consoles        
pico_enable_stdio_usb(pico_cnc_hmi 1)
pico_enable_stdio_uart(pico_cnc_hmi 1)

# Uncomment this line to enable fix for Errata RP2040-E5 (the fix requires use of GPIO 15)
#target_compile_definitions(pico_cnc_hmi PUBLIC PICO_RP2040_USB_DEVICE_ENUMERATION_FIX=1)

pico_add_extra_outputs(pico_cnc_hmi)

# Compile the PIO-programs and include it into the project
pico_generate_pio_header(pico_cnc_hmi ${CMAKE_CURRENT_LIST_DIR}/quadrature.pio)


