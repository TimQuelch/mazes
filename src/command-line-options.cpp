/// \author Tim Quelch

#include <iostream>

#include "command-line-options.h"

namespace mazes {
    CommandLineOptions::CommandLineOptions(int argc, char* argv[]) {
        auto generalOptions = po::options_description{"General"};
        auto mazeOptions = po::options_description{"Maze options"};
        auto videoOptions = po::options_description{"Video options"};

        // clang-format off
        generalOptions.add_options()
            ("help,h", "Print this message");
        mazeOptions.add_options()
            ("maze-size", po::value<unsigned>()->default_value(defaultMazeSize_),
             "The length of the side of the maze")
            ("loop-factor", po::value<double>()->default_value(defaultLoopFactor_),
             "The degree of loopiness in the maze. 0 means there will be a single solution to"
             "the maze, number of solutions increases as value increases");
        videoOptions.add_options()
            ("frame-rate", po::value<unsigned>()->default_value(defaultFrameRate_),
             "The frame rate of produced videos")
            ("pixels-per-tile", po::value<unsigned>()->default_value(defaultPixelsPerTile_),
             "The number of pixels per tile. This should be an even number");
        // clang-format on

        description_.add(generalOptions).add(mazeOptions).add(videoOptions);

        auto vm = po::variables_map{};
        po::store(po::parse_command_line(argc, argv, description_), vm);
        po::notify(vm);

        help_ = vm.count("help");

        mazeSize_ = vm["maze-size"].as<unsigned>();
        loopFactor_ = vm["loop-factor"].as<double>();

        frameRate_ = vm["frame-rate"].as<unsigned>();
        pixelsPerTile_ = vm["pixels-per-tile"].as<unsigned>();

        if (pixelsPerTile_ % 2) {
            std::ostringstream os;
            os << "Invalid number of pixels per tile: " << pixelsPerTile_
               << ". Must be an even number";
            throw std::runtime_error{os.str()};
        }
    }

    void CommandLineOptions::printOptions() const { std::cout << description_ << std::endl; }
} // namespace mazes
