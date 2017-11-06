set(SOURCE_FILES
    mazes.cpp
    maze.cpp)

add_executable(maze ${SOURCE_FILES})
target_link_libraries(maze ${PNG_LIBRARY})

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
