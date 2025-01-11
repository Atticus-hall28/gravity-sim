cmake_minimum_required(VERSION 3.21)
project(gravity VERSION 0.1.0 LANGUAGES C)
set(CMAKE_C_STANDARD 99)
find_package(SDL2 REQUIRED)
add_executable(gravity main.c)
target_link_libraries(gravity SDL2::SDL2)
target_link_libraries(gravity m)


include(CTest)
enable_testing()
