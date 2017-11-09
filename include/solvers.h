#ifndef MAZES_SOLVERS_H
#define MAZES_SOLVERS_H

#include "maze.h"

namespace mazes {
    std::list<std::shared_ptr<Maze::Node>>
    solveBfs(Maze const& maze, std::shared_ptr<Maze::Node> start, std::shared_ptr<Maze::Node> end);

    std::list<std::shared_ptr<Maze::Node>>
    solveDfs(Maze const& maze, std::shared_ptr<Maze::Node> start, std::shared_ptr<Maze::Node> end);

    std::list<std::shared_ptr<Maze::Node>> solveDijkstra(Maze const& maze,
                                                         std::shared_ptr<Maze::Node> start,
                                                         std::shared_ptr<Maze::Node> end);

    std::list<std::shared_ptr<Maze::Node>> solveAstar(Maze const& maze,
                                                      std::shared_ptr<Maze::Node> start,
                                                      std::shared_ptr<Maze::Node> end);
} // namespace mazes

#endif
