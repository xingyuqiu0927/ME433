cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PICO_BOARD pico2_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(HW18 C CXX ASM)

pico_sdk_init()

add_executable(HW18 spring_feedback.c)

pico_set_program_name(HW18 "HW18")
pico_set_program_version(HW18 "0.1")

pico_enable_stdio_uart(HW18 0)
pico_enable_stdio_usb(HW18 1)

target_link_libraries(HW18
        pico_stdlib
        hardware_i2c
        hardware_pwm
        m
        )

target_include_directories(HW18 PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(HW18)
