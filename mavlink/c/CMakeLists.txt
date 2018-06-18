cmake_minimum_required(VERSION 3.6)
project(mavlink C)

set(CMAKE_C_STANDARD 11)

add_executable(mavlink main.c)
include_directories(deps)
