/// \author Tim Quelch

#ifndef MAZES_SOLVERS_H
#define MAZES_SOLVERS_H

#include "maze.h"

namespace mazes {
    /// Solve maze using breadth first search.
    /// \relates Maze
    /// \param maze the Maze to solve
    /// \param start the start Node. This should be a pointer to a Node in the graph of the maze
    /// \param end the end Node. This should be a pointer to a Node in the graph of the maze
    /// \returns A list of nodes that make up the path of the maze, in order of the path
    std::list<std::shared_ptr<Maze::Node>>
    solveBfs(Maze const& maze, std::shared_ptr<Maze::Node> start, std::shared_ptr<Maze::Node> end);

    /// Solve maze using depth first search.
    /// \relates Maze
    /// \copydetails solveBfs()
    std::list<std::shared_ptr<Maze::Node>>
    solveDfs(Maze const& maze, std::shared_ptr<Maze::Node> start, std::shared_ptr<Maze::Node> end);

    /// Solve maze using Dijkstra's algorithm
    /// \relates Maze
    /// \copydetails solveBfs()
    std::list<std::shared_ptr<Maze::Node>> solveDijkstra(Maze const& maze,
                                                         std::shared_ptr<Maze::Node> start,
                                                         std::shared_ptr<Maze::Node> end);

    /// Solve maze using A*
    /// \relates Maze
    /// \copydetails solveBfs()
    std::list<std::shared_ptr<Maze::Node>> solveAstar(Maze const& maze,
                                                      std::shared_ptr<Maze::Node> start,
                                                      std::shared_ptr<Maze::Node> end);
} // namespace mazes

#endif
