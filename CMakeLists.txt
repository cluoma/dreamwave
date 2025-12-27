cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 23)

set(PICO_BOARD pico2_w)
set(PICO_PLATFORM rp2350)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(dreamwave CXX C ASM)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(dreamwave
        src/main.c
        src/maple.c
        src/mp_peripheral.c
        src/mp_peripheral.h
        src/dw_bt_hid.c
        src/dw_bt_hid.h
        src/dw_vmu_static_data.c
        src/dw_vmu_static_data.h
        src/dw_bt_tlv.c
        src/dw_bt_tlv.h
        src/dw_hid_descriptors.c
        src/dw_hid_descriptors.h
)
#target_compile_options(pico2maple PRIVATE -Wall -Wextra -Wpedantic -Werror)
#target_compile_options(pico2maple PRIVATE -O3)

# Add DEBUG definition if building in Debug mode
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_definitions(dreamwave PRIVATE DEBUG)
endif()

# compile PIO programs
pico_generate_pio_header(dreamwave
        ${CMAKE_CURRENT_LIST_DIR}/src/maple_rx.pio
)
pico_generate_pio_header(dreamwave
        ${CMAKE_CURRENT_LIST_DIR}/src/maple_tx.pio
)

# enable uart for debugging with debugprobe
pico_enable_stdio_uart(dreamwave 1)

# create uf2
pico_set_uf2_family(${CMAKE_PROJECT_NAME} "rp2350-arm-s")
pico_add_extra_outputs(dreamwave)

# Make sure TinyUSB can find tusb_config.h
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/src
)

#target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
#        WANT_HCI_DUMP=1 # This enables btstack debug
#)
# Disable LWIP to use cyw43_arch_poll
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        CYW43_LWIP=0
)

# link
string(APPEND CMAKE_EXE_LINKER_FLAGS "-Wl,--print-memory-usage")
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
        pico_stdlib
        pico_multicore
        hardware_pio
        hardware_i2c
        hardware_dma
        pico_cyw43_arch_poll
        pico_btstack_cyw43
        pico_btstack_classic
)
