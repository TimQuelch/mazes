/// \author Tim Quelch

#include <chrono>
#include <iostream>

#include "command-line-options.h"
#include "maze.h"
#include "solvers.h"
#include "video-writer.h"

using hr_clock = std::chrono::high_resolution_clock;

template <typename Solver, typename... Ts>
void runSolution(std::string_view name,
                 std::string_view filename,
                 mazes::CommandLineOptions const& opts,
                 mazes::Maze const& maze,
                 Solver&& solver,
                 Ts... solverArgs) {
    static const auto vidExtension = ".mp4";
    static const auto imgExtension = ".png";

    std::cout << "Solving using " << name << "... " << std::flush;
    const auto start = hr_clock::now();
    std::optional<mazes::VideoWriter> video;
    if (opts.writeVideo()) {
        video = mazes::VideoWriter{maze,
                                   "vid" + std::string{filename} + vidExtension,
                                   opts.frameRate(),
                                   opts.pixelsPerTile(),
                                   opts.nUpdatesPerFrame(),
                                   opts.gradientRate()};
    }
    const auto soln = solver(maze, *maze.getStartNode(), *maze.getEndNode(), video, solverArgs...);
    if (opts.saveSolutionImages()) {
        maze.writePngPath(soln, "img" + std::string{filename} + imgExtension);
    }
    const auto d = std::chrono::duration_cast<std::chrono::milliseconds>(hr_clock::now() - start);
    std::cout << "Time elapsed = " << d.count() << " ms\n";
}

int main(int argc, char* argv[]) {
    auto const opts = mazes::CommandLineOptions{argc, argv};

    if (opts.help()) {
        opts.printOptions();
        return 1;
    }

    std::cout << "Generating maze... " << std::flush;
    auto start = hr_clock::now();
    mazes::Maze maze(opts.mazeSize(), opts.loopFactor(), opts.mazeMethod());
    auto d = std::chrono::duration_cast<std::chrono::milliseconds>(hr_clock::now() - start);
    std::cout << "Time elapsed = " << d.count() << " ms\n";

    if (opts.saveMazeImage()) {
        maze.writePng("maze.png");
    }
    if (opts.solveBfs()) {
        runSolution("BFS", "Bfs", opts, maze, mazes::solveBfs);
    }
    if (opts.solveDfs()) {
        runSolution("DFS", "Dfs", opts, maze, mazes::solveDfs);
    }
    if (opts.solveDijkstra()) {
        runSolution("Dijkstra", "Dij", opts, maze, mazes::solveDijkstra);
    }
    if (opts.solveAstar()) {
        runSolution("A*", "Ast", opts, maze, mazes::solveAstar, opts.astarHeuristicWeighting());
    }
}
