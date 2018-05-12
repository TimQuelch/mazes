/// \author Tim Quelch

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <iomanip>
#include <iostream>
#include <png++/png.hpp>
#include <random>
#include <tuple>

#include "maze.h"

namespace mazes {
    namespace detail {
        /// Holds the x y coordinates of a point in the maze
        struct Point {
            int x; ///< x coordinate
            int y; ///< y coordinate

            /// Construct with given values
            /// \param x The x coordinate
            /// \param y The y coordinate
            Point(int x = 0, int y = 0)
                : x{x}
                , y{y} {}

            /// Orders points lexicographically by x coordinate then y coordinate
            /// \param rhs Another point
            /// \returns true if the point is lexicographically less than the other by x then y
            bool operator<(const Point& rhs) { return std::tie(x, y) < std::tie(rhs.x, rhs.y); }

            /// Points are equal if both x and y are equal
            /// \param rhs Another point
            /// \returns true if both x and y are equal
            bool operator==(const Point& rhs) { return x == rhs.x && y == rhs.y; }
        };

        /// Generates a random integer between a and b
        int randInt(int a, int b) {
            static auto seed = std::chrono::system_clock::now().time_since_epoch().count();
            static auto randEngine = std::default_random_engine{seed};
            auto dist = std::uniform_int_distribution{a, b};
            return dist(randEngine);
        }

        /// Checks if a coordinate is within the boundaries of the maze. That is, if it x and y are
        /// greater than 1 and less than size - 1
        /// \param p A Point
        /// \param gridSize the length of the side of the maze
        /// \returns true if the point is within the walls of the maze
        bool isLegal(Point p, int gridSize) {
            return p.x > 0 && p.x < gridSize - 1 && p.y > 0 && p.y < gridSize - 1;
        }

        /// Checks if point is a wall that can be removed to form a loop. Wall must be a vertical or
        /// horizontal wall, not a T wall, or X wall.
        /// \param grid Grid of the maze
        /// \param p A Point
        /// \returns true if the point can be removed to form a loop
        bool isWall(const std::vector<std::vector<bool>>& grid, Point p) {
            int countWalls = 0;
            countWalls += grid[p.x - 1][p.y] ? 0 : 1;
            countWalls += grid[p.x + 1][p.y] ? 0 : 1;
            countWalls += grid[p.x][p.y - 1] ? 0 : 1;
            countWalls += grid[p.x][p.y + 1] ? 0 : 1;
            bool wall = !grid[p.x][p.y];
            bool nsWall = !grid[p.x][p.y + 1] && !grid[p.x][p.y - 1];
            bool esWall = !grid[p.x - 1][p.y] && !grid[p.x + 1][p.y];
            return wall && (nsWall || esWall) && !(nsWall && esWall) && (countWalls < 3);
        }

        /// Returns a list the points of the neighbours of a point. Neighbours are in the four
        /// cardinal directions and are a distance of two away. They are two away so that the wall
        /// inbetween is jumped.
        std::list<Point> getNeighbours(unsigned gridSize, Point p) {
            auto newNodes = std::list<Point>{};
            newNodes.push_back({p.x - 2, p.y});
            newNodes.push_back({p.x, p.y - 2});
            newNodes.push_back({p.x, p.y + 2});
            newNodes.push_back({p.x + 2, p.y});
            newNodes.remove_if([gridSize](Point p) { return !isLegal(p, gridSize); });
            return newNodes;
        }

        /// Returns and removes a random point from a list
        /// \param points Reference to a list of points. A random point is removed from this list
        /// \returns The Point removed from the list
        Point popPoint(std::list<Point>& points) {
            unsigned index = randInt(0, points.size() - 1);
            auto it = points.begin();
            for (unsigned i = 0; i < index; i++) {
                it++;
            }
            points.erase(it);
            return *it;
        }

        /// Check if a point is a horizontal corridor. That is, it has corridors to the left and
        /// right, and walls to the top and bottom
        bool isHCorridor(Point p, const std::vector<std::vector<bool>>& grid) {
            return grid[p.x - 1][p.y] && grid[p.x + 1][p.y] && !grid[p.x][p.y - 1] &&
                   !grid[p.x][p.y + 1];
        }

        /// Check if a point is a vertical corridor. That is, it has corridors to the top and
        /// bottom, and walls to the left and right
        bool isVCorridor(Point p, const std::vector<std::vector<bool>>& grid) {
            return !grid[p.x - 1][p.y] && !grid[p.x + 1][p.y] && grid[p.x][p.y - 1] &&
                   grid[p.x][p.y + 1];
        }

        /// Calculate the manhatten distance between two points
        int calcCost(Point one, Point two) {
            return std::abs(one.x - two.x) + std::abs(one.y - two.y);
        }

        /// Initialise a grid to false
        std::vector<std::vector<bool>> initGrid(unsigned size) {
            auto grid = std::vector<std::vector<bool>>();
            grid.resize(size);
            for (unsigned i = 0; i < size; i++) {
                grid[i].resize(size, false);
            }
            return grid;
        }

        /// Generate maze with Prims method
        void generatePrims(std::vector<std::vector<bool>>& grid) {
            const auto size = grid.size();

            // Randomly generate initial point
            auto init =
                Point{randInt(0, (size - 2) / 2) * 2 + 1, randInt(0, (size - 2) / 2) * 2 + 1};
            grid[init.x][init.y] = true;

            // Generate frontier points from neighbours
            auto frontierPoints = std::list<Point>{getNeighbours(size, init)};

            // Expand maze until grid is full
            while (!frontierPoints.empty()) {
                // Pick random frontier point and set it to true
                auto p1 = Point{popPoint(frontierPoints)};
                grid[p1.x][p1.y] = true;

                // Pick a random neighbours which is a pathway, and link the two points by setting
                // the intermediate point to a pathway
                auto neighbours = std::list<Point>{getNeighbours(size, p1)};
                neighbours.remove_if([&grid](Point p) { return !grid[p.x][p.y]; });
                auto p2 = Point{popPoint(neighbours)};
                grid[p2.x - (p2.x - p1.x) / 2][p2.y - (p2.y - p1.y) / 2] = true;

                // Find and add the new frontier points
                auto newFrontierPoints = std::list<Point>{getNeighbours(size, p1)};
                newFrontierPoints.remove_if([&grid](Point p) { return grid[p.x][p.y]; });
                frontierPoints.merge(newFrontierPoints);
                frontierPoints.unique();
            }
        }

        std::list<std::pair<Point, Point>> divideChamber(std::vector<std::vector<bool>>& grid,
                                                         std::pair<Point, Point> chamber) {
            const auto x1 = chamber.first.x;
            const auto y1 = chamber.first.y;
            const auto x2 = chamber.second.x;
            const auto y2 = chamber.second.y;
            const auto size = x2 - x1;
            const auto mid = (size + 1) / 2;

            if (size < 2) {
                return {};
            }

            // Set chamber to pathways
            for (auto i = x1; i <= x2; i++) {
                for (auto j = y1; j <= y2; j++) {
                    grid[i][j] = true;
                }
            }

            // Set inner walls to walls
            for (auto i = x1; i <= x2; i++) {
                grid[i][y1 + mid] = false;
            }
            for (auto j = y1; j <= y2; j++) {
                grid[x1 + mid][j] = false;
            }

            // Create openings in walls
            const auto rand = [mid]() { return randInt(0, mid / 2) * 2; };
            auto openings = std::vector<Point>{{x1 + mid, y1 + rand()},
                                               {x1 + mid, y1 + mid + 1 + rand()},
                                               {x1 + rand(), y1 + mid},
                                               {x1 + mid + 1 + rand(), y1 + mid}};
            openings.erase(openings.cbegin() + randInt(0, 3));

            for (auto p : openings) {
                grid[p.x][p.y] = true;
            }

            // Return the subdivided chambers
            return {{{x1, y1}, {x1 + mid - 1, y1 + mid - 1}},
                    {{x1 + mid + 1, y1 + mid + 1}, {x2, y2}},
                    {{x1 + mid + 1, y1}, {x2, y1 + mid - 1}},
                    {{x1, y1 + mid + 1}, {x1 + mid - 1, y2}}};
        }

        /// Generate maze with recursive division method
        void generateDivision(std::vector<std::vector<bool>>& grid) {
            const auto size = static_cast<int>(grid.size());

            auto chambers = std::deque<std::pair<Point, Point>>{};
            chambers.push_back({{1, 1}, {size - 2, size - 2}});

            while (!chambers.empty()) {
                auto newChambers = divideChamber(grid, chambers.front());
                chambers.pop_front();
                for (const auto& c : newChambers) {
                    chambers.push_back(c);
                }
            }
        }

        /// Remove walls to create multiple paths in the maze. Pick random points until a valid
        /// wall is found, then set it to be a pathway
        void addLoops(std::vector<std::vector<bool>>& grid, double loopFactor) {
            const auto size = grid.size();
            const unsigned loops = size * size * loopFactor * loopFactor;
            for (unsigned i = 0; i < loops; i++) {
                Point p;
                do {
                    p = Point(randInt(1, size - 2), randInt(1, size - 2));
                } while (!isWall(grid, p));
                grid[p.x][p.y] = true;
            }
        }

        /// Add the entrance and exit of the maze
        void addEntranceAndExit(std::vector<std::vector<bool>>& grid) {
            const auto size = grid.size();
            grid[1][0] = true;
            grid[size - 2][size - 1] = true;
        }

        /// Generate the grid of a maze. There is an entrance in the top left, and an exit in the
        /// bottom right.
        /// \param size the length of the side of the maze
        /// \param loopFactor the factor of loopiness in the maze. 0 means there is a single
        /// solution, increasing increases number of solutions
        /// \returns the grid of the maze. can be indexed with v[x][y]
        /// \callgraph
        std::vector<std::vector<bool>>
        generateGrid(unsigned size, double loopFactor, Maze::Method method) {
            auto grid = initGrid(size);

            switch (method) {
            case Maze::Method::prims:
                generatePrims(grid);
                break;
            case Maze::Method::division:
                generateDivision(grid);
                break;
            }

            addLoops(grid, loopFactor);
            addEntranceAndExit(grid);

            return grid;
        }

        /// Generate the graph of a maze from the grid. The maze is sorted left to right, then top
        /// to bottom, therefore the entrance is the first node, and the exit is the last node
        /// \param grid The Maze grid to generate the graph from
        /// \returns A list of Nodes that make up the graph of the maze
        std::list<std::shared_ptr<Maze::Node>>
        generateGraph(const std::vector<std::vector<bool>>& grid) {
            using Node = Maze::Node;
            using Edge = Maze::Edge;

            const int size = grid.size();
            auto graph = std::list<std::shared_ptr<Node>>{}; // nodes in the graph
            auto above = std::list<std::shared_ptr<Node>>{}; // nodes that are above current

            // Add start node
            auto first = std::make_shared<Node>(1, 0, std::list<Edge>{});
            graph.push_back(first);
            above.push_back(first);

            // Iterate through whole grid
            for (int j = 1; j < size; j++) {
                auto left = std::shared_ptr<Node>{nullptr}; // The node that is left of present
                auto nodeAbove = above.begin();             // Iterator to start of above nodes
                for (int i = 1; i < size - 1; i++) {
                    if (grid[i][j]) {
                        // Ignore point if it is a corridor
                        if (!isHCorridor({i, j}, grid) && !isVCorridor({i, j}, grid)) {
                            auto edges = std::list<Edge>{};

                            // Add edge if there is a node to the left
                            if (left) {
                                int cost = calcCost({i, j}, {left->x, left->y});
                                edges.push_back({left, cost});
                            }

                            // Add edge if there is a node above
                            if (nodeAbove != above.end() && (*nodeAbove)->x == i) {
                                int cost = calcCost({i, j}, {(*nodeAbove)->x, (*nodeAbove)->y});
                                edges.push_back({*nodeAbove, cost});
                                nodeAbove = above.erase(nodeAbove);
                            }

                            // Create the node
                            auto node = std::make_shared<Node>(i, j, edges);
                            graph.push_back(node);
                            above.insert(nodeAbove, node);
                            left = node;

                            // Add edges to this node from all edges
                            for (const auto& edge : edges) {
                                edge.node->edges.push_back({node, edge.cost});
                            }
                        }

                        // Iterate above iterator if needed
                        if (isVCorridor({i, j}, grid) && nodeAbove != above.end() &&
                            (*nodeAbove)->x == i) {
                            nodeAbove++;
                        }
                    } else {
                        // If current is wall, reset left and iterate above
                        left = nullptr;
                        if (nodeAbove != above.end() && (*nodeAbove)->x == i) {
                            nodeAbove = above.erase(nodeAbove);
                        }
                    }
                }
            }

            // Sort the list of nodes. Entrance will be first, exit will be last
            graph.sort([](auto& one, auto& two) {
                return std::tie(one->x, one->y) < std::tie(two->x, two->y);
            });

            return graph;
        }
    } // namespace detail

    // Construct maze with given size. Generate grid and graph
    Maze::Maze(unsigned size, double loopFactor, Maze::Method method)
        : size_{size}
        , grid_{detail::generateGrid(size, loopFactor, method)}
        , graph_{detail::generateGraph(grid_)} {}

    // Print the maze to stdout
    void Maze::print() const {
        std::cout << "    ";
        for (unsigned i = 0; i < size_; i++) {
            std::cout << std::setw(2) << i << " ";
        }
        std::cout << "\n\n";
        for (unsigned j = 0; j < size_; j++) {
            std::cout << std::setw(2) << j << "  ";
            for (unsigned i = 0; i < size_; i++) {
                if (grid_[i][j]) {
                    std::cout << "   ";
                } else {
                    std::cout << " # ";
                }
            }
            std::cout << "\n";
        }
        std::cout << "\n";
    }

    // Print the graph nodes over the maze to stdout
    void Maze::printGraph() const {
        // Create grid where nodes are true
        auto graphGrid = std::vector<std::vector<bool>>{};
        graphGrid.resize(size_);
        for (unsigned i = 0; i < size_; i++) {
            graphGrid[i].resize(size_, false);
        }
        for (auto nodePtr : graph_) {
            graphGrid[nodePtr->x][nodePtr->y] = true;
        }

        // Print to output
        std::cout << "    ";
        for (unsigned i = 0; i < size_; i++) {
            std::cout << std::setw(2) << i << " ";
        }
        std::cout << "\n\n";
        for (unsigned j = 0; j < size_; j++) {
            std::cout << std::setw(2) << j << "  ";
            for (unsigned i = 0; i < size_; i++) {
                if (graphGrid[i][j]) {
                    std::cout << " o ";
                } else if (grid_[i][j]) {
                    std::cout << "   ";
                } else {
                    std::cout << " # ";
                }
            }
            std::cout << "\n";
        }
        std::cout << "\n";
    }

    // Write the maze to PNG
    void Maze::writePng(std::string_view filename) const {
        auto image = png::image<png::rgb_pixel>{size_, size_};
        for (unsigned j = 0; j < size_; j++) {
            for (unsigned i = 0; i < size_; i++) {
                if (grid_[i][j]) {
                    image[j][i] = png::rgb_pixel(255, 255, 255);
                } else {
                    image[j][i] = png::rgb_pixel(0, 0, 0);
                }
            }
        }
        image.write(filename.data());
    }

    // Write the graph nodes over the maze to PNG
    void Maze::writePngGraph(std::string_view filename) const {
        auto image = png::image<png::rgb_pixel>{size_, size_};

        // Write maze
        for (unsigned j = 0; j < size_; j++) {
            for (unsigned i = 0; i < size_; i++) {
                if (grid_[i][j]) {
                    image[j][i] = png::rgb_pixel(255, 255, 255);
                } else {
                    image[j][i] = png::rgb_pixel(0, 0, 0);
                }
            }
        }

        // Write graph
        for (auto nodePtr : graph_) {
            image[nodePtr->y][nodePtr->x] = png::rgb_pixel(255, 0, 0);
        }

        image.write(filename.data());
    }

    // Write the maze with a given path to PNG
    void Maze::writePngPath(std::list<std::shared_ptr<Node>> path,
                            std::string_view filename) const {
        auto image = png::image<png::rgb_pixel>{size_, size_};

        // Write maze
        for (unsigned j = 0; j < size_; j++) {
            for (unsigned i = 0; i < size_; i++) {
                if (grid_[i][j]) {
                    image[j][i] = png::rgb_pixel(255, 255, 255);
                } else {
                    image[j][i] = png::rgb_pixel(0, 0, 0);
                }
            }
        }

        // Write path
        auto prev = detail::Point{path.front()->x, path.front()->y};
        for (const auto& node : path) {
            auto start = detail::Point{std::min(prev.x, node->x), std::min(prev.y, node->y)};
            auto end =
                detail::Point{std::max(prev.x + 1, node->x + 1), std::max(prev.y + 1, node->y + 1)};
            for (int i = start.x; i != end.x; i++) {
                for (int j = start.y; j != end.y; j++) {
                    image[j][i] = png::rgb_pixel(255, 0, 0);
                }
            }
            prev = {node->x, node->y};
        }

        image.write(filename.data());
    }
} // namespace mazes
