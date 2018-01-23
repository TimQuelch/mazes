/// \author Tim Quelch

#include <chrono>
#include <iostream>

#include "command-line-options.h"
#include "maze.h"
#include "solvers.h"
#include "video-writer.h"

using hr_clock = std::chrono::high_resolution_clock;

int main(int argc, char* argv[]) {
    const auto opts = mazes::CommandLineOptions{argc, argv};

    if (opts.help()) {
        opts.printOptions();
        return 1;
    }

    std::cout << "Generating maze... " << std::flush;
    auto start = hr_clock::now();
    mazes::Maze maze(opts.mazeSize(), opts.loopFactor());
    auto d = std::chrono::duration_cast<std::chrono::milliseconds>(hr_clock::now() - start);
    std::cout << "Time elapsed = " << d.count() << " ms\n";

    if (opts.solveBfs()) {
        std::cout << "Solving using BFS... " << std::flush;
        start = hr_clock::now();
        std::optional<mazes::VideoWriter> video;
        if (opts.writeVideo()) {
            video = mazes::VideoWriter{maze, "vidBfs.webm", opts.frameRate(), opts.pixelsPerTile()};
        }
        auto bfs = solveBfs(maze, maze.getStartNode(), maze.getEndNode(), video);
        d = std::chrono::duration_cast<std::chrono::milliseconds>(hr_clock::now() - start);
        std::cout << "Time elapsed = " << d.count() << " ms\n";
    }

    if (opts.solveDfs()) {
        std::cout << "Solving using DFS... " << std::flush;
        start = hr_clock::now();
        std::optional<mazes::VideoWriter> video;
        if (opts.writeVideo()) {
            video = mazes::VideoWriter{maze, "vidDfs.webm", opts.frameRate(), opts.pixelsPerTile()};
        }
        auto dfs = solveDfs(maze, maze.getStartNode(), maze.getEndNode(), video);
        d = std::chrono::duration_cast<std::chrono::milliseconds>(hr_clock::now() - start);
        std::cout << "Time elapsed = " << d.count() << " ms\n";
    }

    if (opts.solveDijkstra()) {
        std::cout << "Solving using Dijkstra... " << std::flush;
        start = hr_clock::now();
        std::optional<mazes::VideoWriter> video;
        if (opts.writeVideo()) {
            video = mazes::VideoWriter{maze, "vidDij.webm", opts.frameRate(), opts.pixelsPerTile()};
        }
        auto dij = solveDijkstra(maze, maze.getStartNode(), maze.getEndNode(), video);
        d = std::chrono::duration_cast<std::chrono::milliseconds>(hr_clock::now() - start);
        std::cout << "Time elapsed = " << d.count() << " ms\n";
    }

    if (opts.solveAstar()) {
        std::cout << "Solving using A*... " << std::flush;
        start = hr_clock::now();
        std::optional<mazes::VideoWriter> video;
        if (opts.writeVideo()) {
            video = mazes::VideoWriter{maze, "vidAst.webm", opts.frameRate(), opts.pixelsPerTile()};
        }
        auto ast = solveAstar(
            maze, maze.getStartNode(), maze.getEndNode(), video, opts.astarHeuristicWeighting());
        d = std::chrono::duration_cast<std::chrono::milliseconds>(hr_clock::now() - start);
        std::cout << "Time elapsed = " << d.count() << " ms\n";
    }
}
