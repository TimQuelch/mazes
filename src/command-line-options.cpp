/// \author Tim Quelch

#include <iostream>

#include "command-line-options.h"

namespace mazes {
    CommandLineOptions::CommandLineOptions(int argc, char* argv[]) {
        auto generalOptions = po::options_description{"General"};
        auto mazeOptions = po::options_description{"Maze options"};
        auto videoOptions = po::options_description{"Video options"};
        auto imageOptions = po::options_description{"Image options"};
        auto solverOptions = po::options_description{"Solver options"};

        // clang-format off
        generalOptions.add_options()
            ("help,h", "Print this message");
        mazeOptions.add_options()
            ("maze-size", po::value<unsigned>()->default_value(defaultMazeSize_),
             "The length of the side of the maze")
            ("loop-factor", po::value<double>()->default_value(defaultLoopFactor_),
             "The degree of loopiness in the maze. 0 means there will be a single solution to"
             "the maze, number of solutions increases as value increases");
        solverOptions.add_options()
            ("solve-all", "solve maze using all algorithms (other solve flags are ignored)")
            ("solve-bfs", "solve maze using Breadth First Search")
            ("solve-dfs", "solve maze using Depth First Search")
            ("solve-dijkstra", "solve maze using Dijkstra's Algorithm")
            ("solve-astar", "solve maze using A*")
            ("astar-heuristic-weighting",
             po::value<double>()->default_value(defaultAstarHeuristicWeighting_),
             "weighting to use for heuristic in A* solver. Values greater than 1 will find"
             "a solution more quickly, but not necessarily the shortest path");
        videoOptions.add_options()
            ("write-video", "write video solutions to files")
            ("frame-rate", po::value<unsigned>()->default_value(defaultFrameRate_),
             "The frame rate of produced videos")
            ("pixels-per-tile", po::value<unsigned>()->default_value(defaultPixelsPerTile_),
             "The number of pixels per tile. This should be an even number");
        imageOptions.add_options()
            ("save-maze", "save maze to image file");
        // clang-format on

        description_.add(generalOptions)
            .add(mazeOptions)
            .add(solverOptions)
            .add(videoOptions)
            .add(imageOptions);

        auto vm = po::variables_map{};
        po::store(po::parse_command_line(argc, argv, description_), vm);
        po::notify(vm);

        help_ = vm.count("help");
        solveBfs_ = vm.count("solve-bfs") || vm.count("solve-all");
        solveDfs_ = vm.count("solve-dfs") || vm.count("solve-all");
        solveDijkstra_ = vm.count("solve-dijkstra") || vm.count("solve-all");
        solveAstar_ = vm.count("solve-astar") || vm.count("solve-all");

        astarHeuristicWeighting_ = vm["astar-heuristic-weighting"].as<double>();

        mazeSize_ = vm["maze-size"].as<unsigned>();
        loopFactor_ = vm["loop-factor"].as<double>();

        writeVideo_ = vm.count("write-video");
        frameRate_ = vm["frame-rate"].as<unsigned>();
        pixelsPerTile_ = vm["pixels-per-tile"].as<unsigned>();

        saveMazeImage_ = vm.count("save-maze");

        if (pixelsPerTile_ % 2) {
            std::ostringstream os;
            os << "Invalid number of pixels per tile: " << pixelsPerTile_
               << ". Must be an even number";
            throw std::runtime_error{os.str()};
        }
    }

    void CommandLineOptions::printOptions() const { std::cout << description_ << std::endl; }
} // namespace mazes
