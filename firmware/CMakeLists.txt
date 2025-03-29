cmake_minimum_required(VERSION 3.15)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# LVGL in particular is very slow to compile and upload, so this is an easy way to turn it off
# for developing other parts of the system more quickly
option(ENABLE_DISPLAY "Enable SPI OLED display support" OFF)

# allow USB to be enabled/disabled
option(ENABLE_USB "Enable USB" OFF)


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

# Set flags and directory variables
set(FREERTOS_CFG_DIRECTORY "${CMAKE_SOURCE_DIR}")
set(FREERTOS_SRC_DIRECTORY "${CMAKE_SOURCE_DIR}/FreeRTOS-Kernel")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)
project(pico_cnc_hmi C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add FreeRTOS as a library
add_library(FreeRTOS STATIC
    ${FREERTOS_SRC_DIRECTORY}/event_groups.c
    ${FREERTOS_SRC_DIRECTORY}/list.c
    ${FREERTOS_SRC_DIRECTORY}/queue.c
    ${FREERTOS_SRC_DIRECTORY}/stream_buffer.c
    ${FREERTOS_SRC_DIRECTORY}/tasks.c
    ${FREERTOS_SRC_DIRECTORY}/timers.c
    ${FREERTOS_SRC_DIRECTORY}/portable/MemMang/heap_3.c
    ${FREERTOS_SRC_DIRECTORY}/portable/GCC/ARM_CM0/port.c
)

# Build FreeRTOS
target_include_directories(FreeRTOS PUBLIC
    ${FREERTOS_CFG_DIRECTORY}/
    ${FREERTOS_SRC_DIRECTORY}/include
    ${FREERTOS_SRC_DIRECTORY}/portable/GCC/ARM_CM0
)

# Add executable. Default name is the project name, version 0.1
add_executable(pico_cnc_hmi)


# define sources and libs for display/LVGL support
if(ENABLE_DISPLAY)
    set(display_sources 
        ${CMAKE_CURRENT_LIST_DIR}/spi.cc
        ${CMAKE_CURRENT_LIST_DIR}/oled_sh1122.cc
        )

    set(display_libs
        hardware_spi
        lvgl
        )
else()
    set(display_sources "")
    set(display_libs "")
endif()

if(ENABLE_USB)
    set(usb_sources
        ${CMAKE_CURRENT_LIST_DIR}/usb_descriptors.c
        )
    set(usb_libs
        tinyusb_device
        tinyusb_board 
        )
else()
    set(usb_sources "")
    set(usb_libs "")
endif()

target_sources(pico_cnc_hmi PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/main.cc
        ${CMAKE_CURRENT_LIST_DIR}/encoder.cc
        ${CMAKE_CURRENT_LIST_DIR}/i2c.cc
        ${CMAKE_CURRENT_LIST_DIR}/tca8418.cc
        ${CMAKE_CURRENT_LIST_DIR}/tlc59116.cc
        ${CMAKE_CURRENT_LIST_DIR}/ws2812.cc
        ${CMAKE_CURRENT_LIST_DIR}/task_encoder.cc
        ${CMAKE_CURRENT_LIST_DIR}/task_matrix.cc
        ${CMAKE_CURRENT_LIST_DIR}/task_led.cc
        ${display_sources}
        ${usb_sources}
        )

# Make sure TinyUSB can find tusb_config.h
target_include_directories(pico_cnc_hmi PUBLIC
        ${CMAKE_CURRENT_LIST_DIR})

if(ENABLE_DISPLAY)
# copy our modified lv_conf.h into the right place so the build finds it
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/lv_conf.h DESTINATION ${CMAKE_BINARY_DIR}/lvgl)
add_subdirectory(lvgl)
endif()

# In addition to pico_stdlib required for common PicoSDK functionality, add dependency on tinyusb_device
# for TinyUSB device support and tinyusb_board for the additional board support library used by the example
target_link_libraries(pico_cnc_hmi PUBLIC pico_stdlib pico_stdio pico_unique_id FreeRTOS hardware_pio hardware_i2c ${display_libs} ${usb_libs})

# stdio serial consoles        
pico_enable_stdio_usb(pico_cnc_hmi 1)
pico_enable_stdio_uart(pico_cnc_hmi 1)

# Uncomment this line to enable fix for Errata RP2040-E5 (the fix requires use of GPIO 15)
#target_compile_definitions(pico_cnc_hmi PUBLIC PICO_RP2040_USB_DEVICE_ENUMERATION_FIX=1)

pico_add_extra_outputs(pico_cnc_hmi)

# Compile the PIO programs and include them into the project
pico_generate_pio_header(pico_cnc_hmi ${CMAKE_CURRENT_LIST_DIR}/quadrature.pio)
pico_generate_pio_header(pico_cnc_hmi ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)


