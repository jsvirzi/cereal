cmake_minimum_required(VERSION 3.14)
project(cereal C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "../lib")

include_directories(inc)

add_library(cereal SHARED src/cereal.c inc/cereal.h)
