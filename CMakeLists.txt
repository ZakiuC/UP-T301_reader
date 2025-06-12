cmake_minimum_required(VERSION 3.0)
project(laser_reader)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra -Wpedantic)


add_executable(main
    main.cpp
    LaserSerial.cpp
    LaserHeightCalculator.cpp
)

find_package(Threads REQUIRED)
target_link_libraries(main PRIVATE Threads::Threads)
