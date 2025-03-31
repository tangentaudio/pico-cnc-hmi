cmake_minimum_required(VERSION 3.15)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_CXX_ENABLE_EXCEPTIONS 1)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# LVGL in particular is very slow to compile and upload, so this is an easy way to turn it off
# for developing other parts of the system more quickly
option(ENABLE_DISPLAY "Enable SPI OLED display support" ON)

# allow USB to be enabled/disabled
option(ENABLE_USB "Enable USB" ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
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



SET(FREERTOS_KERNEL_PATH "${CMAKE_CURRENT_LIST_DIR}/FreeRTOS" CACHE STRING "FreeRTOS pico kernel port path")
SET(FREERTOS_CONFIG_FILE_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/port/FreeRTOS-Kernel" CACHE STRING "Local FreeRTOS Config")
include("${FREERTOS_CONFIG_FILE_DIRECTORY}/freertos_config.cmake") 
include(FreeRTOS_Kernel_import.cmake)


# Add executable. Default name is the project name, version 0.1
add_executable(pico_cnc_hmi)

# define sources and libs for display/LVGL support
if(ENABLE_DISPLAY)
    set(display_sources 
        ${CMAKE_CURRENT_LIST_DIR}/drivers/spi.cc
        ${CMAKE_CURRENT_LIST_DIR}/drivers/oled_sh1122.cc
        ${CMAKE_CURRENT_LIST_DIR}/fonts/roboto_64.c
        ${CMAKE_CURRENT_LIST_DIR}/task_display.cc
        )

    set(display_libs
        hardware_spi
        lvgl
        )
    
    add_definitions(-DENABLE_DISPLAY)
else()
    set(display_sources "")
    set(display_libs "")
endif()

if(ENABLE_USB)
    set(usb_sources
        ${CMAKE_CURRENT_LIST_DIR}/port/tinyusb/usb_descriptors.c
        ${CMAKE_CURRENT_LIST_DIR}/drivers/usb.c
        )
    set(usb_libs
        tinyusb_device
        tinyusb_board 
        )
    add_definitions(-DENABLE_USB)
else()
    set(usb_sources "")
    set(usb_libs "")
endif()

target_sources(pico_cnc_hmi PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/main.cc
        ${CMAKE_CURRENT_LIST_DIR}/drivers/encoder.cc
        ${CMAKE_CURRENT_LIST_DIR}/drivers/i2c.cc
        ${CMAKE_CURRENT_LIST_DIR}/drivers/tca8418.cc
        ${CMAKE_CURRENT_LIST_DIR}/drivers/tlc59116.cc
        ${CMAKE_CURRENT_LIST_DIR}/drivers/ws2812.cc
        ${CMAKE_CURRENT_LIST_DIR}/task_encoder.cc
        ${CMAKE_CURRENT_LIST_DIR}/task_matrix.cc
        ${CMAKE_CURRENT_LIST_DIR}/task_led.cc
        ${display_sources}
        ${usb_sources}
        )

target_include_directories(pico_cnc_hmi PUBLIC
if(ENABLE_USB)
        ${CMAKE_CURRENT_LIST_DIR}/port/tinyusb
endif()
        ${CMAKE_CURRENT_LIST_DIR}/drivers
        ${FREERTOS_CONFIG_FILE_DIRECTORY}
        ${FREERTOS_KERNEL_PATH}/include
        ${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2350_ARM_NTZ/non_secure)

if(ENABLE_DISPLAY)
# copy our modified lv_conf.h into the right place so the build finds it
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/port/lvgl/lv_conf.h DESTINATION ${CMAKE_BINARY_DIR}/lvgl)

add_subdirectory(lvgl)

target_include_directories(lvgl PUBLIC
    # clearly this is insane but it works until I can figure out how to properly include the Pico SDK paths for LVGL
    ${CMAKE_CURRENT_BINARY_DIR}/generated/pico_base/
    ${PICO_SDK_PATH}/src/common/pico_base_headers/include/
    ${PICO_SDK_PATH}/src/rp2_common/hardware_sync/include/
    ${PICO_SDK_PATH}/src/rp2350/pico_platform/include/
    ${PICO_SDK_PATH}/src/rp2350/hardware_regs/include/
    ${PICO_SDK_PATH}/src/rp2_common/pico_platform_sections/include/
    ${PICO_SDK_PATH}/src/rp2_common/pico_platform_compiler/include/
    ${PICO_SDK_PATH}/src/rp2_common/pico_platform_panic/include/
    ${PICO_SDK_PATH}/src/rp2_common/hardware_base/include/
    ${PICO_SDK_PATH}/src/rp2350/hardware_structs/include/
    ${PICO_SDK_PATH}/src/rp2_common/hardware_sync_spin_lock/include/
    # needed for FreeRTOS LVGL build
    ${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2350_ARM_NTZ/non_secure
    ${FREERTOS_CONFIG_FILE_DIRECTORY}
    ${FREERTOS_KERNEL_PATH}/include
)



endif()

target_link_libraries(pico_cnc_hmi PUBLIC pico_stdlib pico_stdio_uart pico_unique_id hardware_pio hardware_i2c FreeRTOS-Kernel-Heap4 freertos_config ${display_libs} ${usb_libs})

# stdio serial consoles        
pico_enable_stdio_usb(pico_cnc_hmi 0)
pico_enable_stdio_uart(pico_cnc_hmi 1)

# Uncomment this line to enable fix for Errata RP2040-E5 (the fix requires use of GPIO 15)
#target_compile_definitions(pico_cnc_hmi PUBLIC PICO_RP2040_USB_DEVICE_ENUMERATION_FIX=1)

pico_add_extra_outputs(pico_cnc_hmi)

# Compile the PIO programs and include them into the project
pico_generate_pio_header(pico_cnc_hmi ${CMAKE_CURRENT_LIST_DIR}/drivers/quadrature.pio)
pico_generate_pio_header(pico_cnc_hmi ${CMAKE_CURRENT_LIST_DIR}/drivers/ws2812.pio)


