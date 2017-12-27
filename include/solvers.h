/// \author Tim Quelch

#ifndef MAZES_SOLVERS_H
#define MAZES_SOLVERS_H

#include "maze.h"
#include "video-writer.h"

namespace mazes {
    /// Solve maze using breadth first search.
    /// \param maze the Maze to solve
    /// \param start the start Node. This should be a pointer to a Node in the graph of the maze
    /// \param end the end Node. This should be a pointer to a Node in the graph of the maze
    /// \returns A list of nodes that make up the path of the maze, in order of the path
    /// \relates Maze
    std::list<std::shared_ptr<Maze::Node>>
    solveBfs(Maze const& maze, std::shared_ptr<Maze::Node> start, std::shared_ptr<Maze::Node> end);

    /// \copydoc solveBfs(Maze const&,std::shared_ptr<Maze::Node>,std::shared_ptr<Maze::Node>)
    /// \relates Maze
    /// This overload also writes the solution to a specified video file
    /// \param filename file to write to a video
    std::list<std::shared_ptr<Maze::Node>> solveBfs(Maze const& maze,
                                                    std::shared_ptr<Maze::Node> start,
                                                    std::shared_ptr<Maze::Node> end,
                                                    std::string_view filename);

    /// Solve maze using depth first search.
    /// \copydetails solveBfs()
    /// \relates Maze
    std::list<std::shared_ptr<Maze::Node>>
    solveDfs(Maze const& maze, std::shared_ptr<Maze::Node> start, std::shared_ptr<Maze::Node> end);

    /// \copydoc solveDfs(Maze const&,std::shared_ptr<Maze::Node>,std::shared_ptr<Maze::Node>)
    /// \relates Maze
    /// This overload also writes the solution to a specified video file
    /// \param filename file to write to a video
    std::list<std::shared_ptr<Maze::Node>> solveDfs(Maze const& maze,
                                                    std::shared_ptr<Maze::Node> start,
                                                    std::shared_ptr<Maze::Node> end,
                                                    std::string_view filename);

    /// Solve maze using Dijkstra's algorithm
    /// \copydetails solveBfs()
    /// \relates Maze
    std::list<std::shared_ptr<Maze::Node>> solveDijkstra(Maze const& maze,
                                                         std::shared_ptr<Maze::Node> start,
                                                         std::shared_ptr<Maze::Node> end);

    /// \copydoc solveDijkstra(Maze const&,std::shared_ptr<Maze::Node>,std::shared_ptr<Maze::Node>)
    /// \relates Maze
    /// This overload also writes the solution to a specified video file
    /// \param filename file to write to a video
    std::list<std::shared_ptr<Maze::Node>> solveDijkstra(Maze const& maze,
                                                         std::shared_ptr<Maze::Node> start,
                                                         std::shared_ptr<Maze::Node> end,
                                                         std::string_view filename);

    /// Solve maze using A*
    /// \copydetails solveBfs()
    /// \relates Maze
    std::list<std::shared_ptr<Maze::Node>> solveAstar(Maze const& maze,
                                                      std::shared_ptr<Maze::Node> start,
                                                      std::shared_ptr<Maze::Node> end);

    /// \copydoc solveAstar(Maze const&,std::shared_ptr<Maze::Node>,std::shared_ptr<Maze::Node>)
    /// \relates Maze
    /// This overload also writes the solution to a specified video file
    /// \param filename file to write to a video
    std::list<std::shared_ptr<Maze::Node>> solveAstar(Maze const& maze,
                                                      std::shared_ptr<Maze::Node> start,
                                                      std::shared_ptr<Maze::Node> end,
                                                      std::string_view filename);
} // namespace mazes

#endif
