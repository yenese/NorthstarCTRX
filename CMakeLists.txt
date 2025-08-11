# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ctrx)

target_sources(app PRIVATE
    src/main.c
    src/my_spi.c
    src/nrf21540.c
    src/my_usb.c
    src/my_esb.c
    src/led.c
    src/systime.c
    src/rf52.c
    src/Northlib/ntrp.c
    src/Northlib/ntrprouter.c
)

target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Northlib
)
