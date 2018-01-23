/// \author Tim Quelch

#ifndef MAZES_SOLVERS_H
#define MAZES_SOLVERS_H

#include "maze.h"

#include <optional>
#include <string_view>

namespace mazes {
    class VideoWriter;

    /// Solve maze using breadth first search.
    /// \param maze the Maze to solve
    /// \param start the start Node. This should be a pointer to a Node in the graph of the maze
    /// \param end the end Node. This should be a pointer to a Node in the graph of the maze
    /// \param video optional video writer to write the solution to
    /// \returns A list of nodes that make up the path of the maze, in order of the path
    /// \relates Maze
    std::list<std::shared_ptr<Maze::Node>> solveBfs(Maze const& maze,
                                                    std::shared_ptr<Maze::Node> start,
                                                    std::shared_ptr<Maze::Node> end,
                                                    std::optional<VideoWriter>& video);

    /// Solve maze using depth first search.
    /// \copydetails solveBfs()
    /// \relates Maze
    std::list<std::shared_ptr<Maze::Node>> solveDfs(Maze const& maze,
                                                    std::shared_ptr<Maze::Node> start,
                                                    std::shared_ptr<Maze::Node> end,
                                                    std::optional<VideoWriter>& video);

    /// Solve maze using Dijkstra's algorithm
    /// \copydetails solveBfs()
    /// \relates Maze
    std::list<std::shared_ptr<Maze::Node>> solveDijkstra(Maze const& maze,
                                                         std::shared_ptr<Maze::Node> start,
                                                         std::shared_ptr<Maze::Node> end,
                                                         std::optional<VideoWriter>& video);

    /// Solve maze using A*
    /// \param heuristicWeighting the weighting to give to the heuristic
    /// \copydetails solveBfs()
    /// \relates Maze
    std::list<std::shared_ptr<Maze::Node>> solveAstar(Maze const& maze,
                                                      std::shared_ptr<Maze::Node> start,
                                                      std::shared_ptr<Maze::Node> end,
                                                      std::optional<VideoWriter>& video,
                                                      double heuristicWeighting);
} // namespace mazes

#endif
