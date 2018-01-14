/// \author Tim Quelch

#ifndef MAZE_COMMAND_LINE_OPTIONS_H
#define MAZE_COMMAND_LINE_OPTIONS_H

#include <boost/program_options.hpp>
namespace po = boost::program_options;

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

        /// Get the maze size
        /// \returns The maze size
        unsigned mazeSize() const { return mazeSize_; }
        /// Get the loop factor
        /// \returns The loop factor
        double loopFactor() const { return loopFactor_; }

        /// Get the frame rate
        /// \returns The frame rate
        unsigned frameRate() const { return frameRate_; }
        /// Get the number of pixels per tile
        /// \returns The number of pixels per tile
        unsigned pixelsPerTile() const { return pixelsPerTile_; }

    private:
        /// The options description
        po::options_description description_{"Usage"};

        bool help_; ///< True if the help flag is passed

        unsigned mazeSize_; ///< The maze size
        double loopFactor_; ///< The loop factor

        static constexpr unsigned defaultMazeSize_{21};  ///< The default maze size
        static constexpr double defaultLoopFactor_{0.0}; ///< The default loop factor

        unsigned frameRate_;     ///< The frame rate
        unsigned pixelsPerTile_; ///< The number of pixels per tile

        static constexpr unsigned defaultFrameRate_{60};    ///< The default frame rate
        static constexpr unsigned defaultPixelsPerTile_{2}; ///< The default pixels per tile
    };
} // namespace mazes

#endif
