cmake_minimum_required(VERSION 3.8)
project(DCAS)

set(CMAKE_C_STANDARD 99)

set(SOURCE_FILES dcas.c map.h attribute.h drone.h drone.c map.c)
add_executable(DCAS ${SOURCE_FILES})
