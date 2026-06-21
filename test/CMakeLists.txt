cmake_minimum_required(VERSION 3.13)
project(simtest C)

add_executable(simtest
    simtest.c
    ../ballistics/atmosphere.c
    ../ballistics/drag.c
    ../ballistics/gun_data.c
    ../ballistics/integrator.c
    ../ballistics/solver.c
)

target_include_directories(simtest PRIVATE ..)
target_link_libraries(simtest m)
