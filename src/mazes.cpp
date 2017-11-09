#include "maze.h"
#include "solvers.h"

int main() {
    mazes::Maze maze(31, 0.15);
    maze.print();
    maze.printGraph();
    maze.writePng("maze.png");
    maze.writePngGraph("graph.png");
    auto bfs = solveBfs(maze, maze.getStartNode(), maze.getEndNode());
    auto dfs = solveDfs(maze, maze.getStartNode(), maze.getEndNode());
    maze.writePngPath(bfs, "path-bfs.png");
    maze.writePngPath(dfs, "path-dfs.png");
    return 0;
}
