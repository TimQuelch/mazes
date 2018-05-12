cmake_minimum_required(VERSION 3.10)
project(mazes VERSION 1.0.0 LANGUAGES CXX)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake)
find_package(png++ REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(FFmpeg REQUIRED)

add_executable(
    mazes
    src/mazes.cpp
    src/solvers.cpp
    src/video-writer.cpp
    src/command-line-options.cpp
    src/maze.cpp)
target_include_directories(mazes PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(mazes PRIVATE cxx_std_17)
target_compile_options(mazes PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(
    mazes
    PRIVATE
    Boost::program_options
    FFmpeg::avcodec
    FFmpeg::avutil
    FFmpeg::avformat
    FFmpeg::swscale
    png++::png++)

find_package(Doxygen)
if (DOXYGEN_FOUND)
    set(DOXYGEN_QUIET YES)
    set(DOXYGEN_WARNINGS YES)
    set(DOXYGEN_WARN_NO_PARAMDOC YES)
    set(DOXYGEN_EXTRACT_PRIVATE YES)
    set(DOXYGEN_EXTRACT_STATIC YES)
    set(DOXYGEN_BUILTIN_STL_SUPPORT YES)
    set(DOXYGEN_GENERATE_TREEVIEW YES)
    doxygen_add_docs(docs)
endif()

include(GNUInstallDirs)
install(
    TARGETS mazes
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
