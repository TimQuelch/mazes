/// \author Tim Quelch

#ifndef MAZE_COMMAND_LINE_OPTIONS_H
#define MAZE_COMMAND_LINE_OPTIONS_H

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include "maze.h"

namespace mazes {
    /// Parse and store command line options for the maze generation and solutions
    class CommandLineOptions {
    public:
        /// Construct from the arguments passed from the main function
        /// \param argc The number of arguments
        /// \param argv Pointer to strings of command line arguments
        CommandLineOptions(int argc, char* argv[]);

        /// Print help message to stdout
        void printOptions() const;

        /// Get whether the help flag was passed
        /// \returns true if help flag is passed
        bool help() const { return help_; }

        /// Get whether the solve-bfs flag was passed
        /// \returns true if solve-bfs flag is passed
        bool solveBfs() const { return solveBfs_; }
        /// Get whether the solve-dfs flag was passed
        /// \returns true if solve-dfs flag is passed
        bool solveDfs() const { return solveDfs_; }
        /// Get whether the solve-dijkstra flag was passed
        /// \returns true if solve-dijkstra flag is passed
        bool solveDijkstra() const { return solveDijkstra_; }
        /// Get whether the solve-astar flag was passed
        /// \returns true if solve-astar flag is passed
        bool solveAstar() const { return solveAstar_; }

        /// Gets the weighting for the A* heuristic
        /// \returns the weighting of the A* heuristic
        double astarHeuristicWeighting() const { return astarHeuristicWeighting_; }

        /// Get the maze size
        /// \returns The maze size
        unsigned mazeSize() const { return mazeSize_; }
        /// Get the loop factor
        /// \returns The loop factor
        double loopFactor() const { return loopFactor_; }
        /// Get the method to use to generate the maze
        /// \returns The maze generation method
        Maze::Method mazeMethod() const { return mazeMethod_; }

        /// Get whether the write-video flag was passed
        /// \returns true if write-video flag is passed
        bool writeVideo() const { return writeVideo_; }
        /// Get the frame rate
        /// \returns The frame rate
        unsigned frameRate() const { return frameRate_; }
        /// Get the number of pixels per tile
        /// \returns The number of pixels per tile
        unsigned pixelsPerTile() const { return pixelsPerTile_; }
        /// Get the number of updates per frame
        /// \returns The number updates per frame
        unsigned nUpdatesPerFrame() const { return nUpdatesPerFrame_; }

        /// Get whether the save-maze flag was passed
        /// \returns true if save-maze flag is passed
        bool saveMazeImage() const { return saveMazeImage_; }

    private:
        /// The options description
        po::options_description description_{"Usage"};

        bool help_; ///< True if the help flag is passed

        bool solveBfs_;      ///< True if the solve-bfs flag is passed
        bool solveDfs_;      ///< True if the solve-dfs flag is passed
        bool solveDijkstra_; ///< True if the solve-dijkstra flag is passed
        bool solveAstar_;    ///< True if the solve-astar flag is passed

        double astarHeuristicWeighting_; ///< Weighting for A* heuristic

        /// The default weighting heuristic
        static constexpr double defaultAstarHeuristicWeighting_{1.0};

        unsigned mazeSize_;       ///< The maze size
        double loopFactor_;       ///< The loop factor
        Maze::Method mazeMethod_; ///< The maze generation method

        static constexpr unsigned defaultMazeSize_{21};  ///< The default maze size
        static constexpr double defaultLoopFactor_{0.0}; ///< The default loop factor
        static const std::string defaultMazeMethod_;     ///< The default loop factor

        bool writeVideo_;           ///< True if the write-video flag is passed
        unsigned frameRate_;        ///< The frame rate
        unsigned pixelsPerTile_;    ///< The number of pixels per tile
        unsigned nUpdatesPerFrame_; ///< The number of updates per frame

        static constexpr unsigned defaultFrameRate_{60};       ///< The default frame rate
        static constexpr unsigned defaultPixelsPerTile_{2};    ///< The default pixels per tile
        static constexpr unsigned defaultNUpdatesPerFrame_{2}; ///< The default updates per frame

        bool saveMazeImage_; ///< True if the maze should be saved as an image
    };
} // namespace mazes

#endif
