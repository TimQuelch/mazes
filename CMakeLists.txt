cmake_minimum_required(VERSION 3.7)
project(mazes VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -O3 -pipe -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -Wextra -Wpedantic")

find_package(PNG REQUIRED)

find_library(FFMPEG_LIB_avformat avformat /usr/lib)
find_library(FFMPEG_LIB_avcodec avcodec /usr/lib)
find_library(FFMPEG_LIB_avutil avutil /usr/lib)
find_library(FFMPEG_LIB_swscale swscale /usr/lib)
find_library(FFMPEG_LIB_avdevice avdevice /usr/lib)

include_directories(include)

add_subdirectory(src)
add_subdirectory(docs)
