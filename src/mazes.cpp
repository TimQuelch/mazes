#include "maze.h"

int main() {
    mazes::Maze maze(31, 0.15);
    maze.print();
    maze.printGraph();

    return 0;
}
