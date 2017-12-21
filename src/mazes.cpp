/// \author Tim Quelch

#include <chrono>
#include <iostream>

#include "maze.h"
#include "solvers.h"

using hr_clock = std::chrono::high_resolution_clock;

int main() {
    std::cout << "Generating maze... " << std::flush;
    auto start = hr_clock::now();
    mazes::Maze maze(201, 0.10);
    auto d = std::chrono::duration_cast<std::chrono::milliseconds>(hr_clock::now() - start);
    std::cout << "Time elapsed = " << d.count() << " ms\n";

    std::cout << "Solving using BFS... " << std::flush;
    start = hr_clock::now();
    auto bfs = solveBfs(maze, maze.getStartNode(), maze.getEndNode());
    d = std::chrono::duration_cast<std::chrono::milliseconds>(hr_clock::now() - start);
    std::cout << "Time elapsed = " << d.count() << " ms\n";

    std::cout << "Solving using DFS... " << std::flush;
    start = hr_clock::now();
    auto dfs = solveDfs(maze, maze.getStartNode(), maze.getEndNode());
    d = std::chrono::duration_cast<std::chrono::milliseconds>(hr_clock::now() - start);
    std::cout << "Time elapsed = " << d.count() << " ms\n";

    std::cout << "Solving using Dijkstra... " << std::flush;
    start = hr_clock::now();
    auto dij = solveDijkstra(maze, maze.getStartNode(), maze.getEndNode());
    d = std::chrono::duration_cast<std::chrono::milliseconds>(hr_clock::now() - start);
    std::cout << "Time elapsed = " << d.count() << " ms\n";

    std::cout << "Solving using A*... " << std::flush;
    start = hr_clock::now();
    auto ast = solveAstar(maze, maze.getStartNode(), maze.getEndNode());
    d = std::chrono::duration_cast<std::chrono::milliseconds>(hr_clock::now() - start);
    std::cout << "Time elapsed = " << d.count() << " ms\n";

    maze.writePng("maze.png");
    maze.writePngGraph("graph.png");
    maze.writePngPath(bfs, "path-bfs.png");
    maze.writePngPath(dfs, "path-dfs.png");
    maze.writePngPath(dij, "path-dij.png");
    maze.writePngPath(ast, "path-ast.png");
    return 0;
}
