cmake_minimum_required(VERSION 3.10)

project(path-finding-visualizer)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

file(GLOB SOURCES *.cc algorithms/*.cc)
add_executable(path-finding-visualizer ${SOURCES})

find_package(SDL2_image)
find_package(SDL2_ttf)
find_package(SDL2)
find_package(spdlog)
find_package(argparse)

target_link_libraries(
    path-finding-visualizer SDL2_image::SDL2_image sdl_ttf::sdl_ttf
    SDL2::SDL2main spdlog::spdlog argparse::argparse)
