set(SOURCE_FILES
    mazes.cpp
    solvers.cpp
    video-writer.cpp
    command-line-options.cpp
    maze.cpp)

add_executable(maze ${SOURCE_FILES})
target_link_libraries(maze ${PNG_LIBRARY} ${FFMPEG_LIB_avcodec} ${FFMPEG_LIB_avutil} ${FFMPEG_LIB_avformat} ${FFMPEG_LIB_swscale} ${Boost_PROGRAM_OPTIONS_LIBRARY})

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
