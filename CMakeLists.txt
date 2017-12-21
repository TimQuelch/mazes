cmake_minimum_required(VERSION 3.7)
project(mazes VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -O3 -pipe -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -Wpedantic")

find_package(PNG REQUIRED)

include_directories(include)

add_subdirectory(src)
add_subdirectory(docs)
