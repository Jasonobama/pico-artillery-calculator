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

project(jsqpico2 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(jsqpico2
    main.c
    ui/pico_keypad4x4.c
    ui/lcd1602_i2c.c
    ui/menu.c
    ballistics/gun_data.c
    ballistics/atmosphere.c
    ballistics/drag.c
    ballistics/integrator.c
    ballistics/solver.c
    core1/core1_task.c
)

pico_set_program_name(jsqpico2 "jsqpico2")
pico_set_program_version(jsqpico2 "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(jsqpico2 0)
pico_enable_stdio_usb(jsqpico2 1)

# Add the standard library to the build
target_link_libraries(jsqpico2
        pico_stdlib
        pico_multicore
)

# Add the standard include files to the build
target_include_directories(jsqpico2 PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/ui
        ${CMAKE_CURRENT_LIST_DIR}/ballistics
        ${CMAKE_CURRENT_LIST_DIR}/core1
)

# Add any user requested libraries
target_link_libraries(jsqpico2
        hardware_i2c
        hardware_gpio
)

pico_add_extra_outputs(jsqpico2)
