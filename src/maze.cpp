#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <list>
#include <png++/png.hpp>
#include <random>
#include <tuple>

#include "maze.h"

namespace mazes {
    namespace {
        struct Point {
            int x;
            int y;

            Point(int x = 0, int y = 0)
                : x{x}
                , y{y} {}

            bool operator<(const Point& rhs) { return std::tie(x, y) < std::tie(rhs.x, rhs.y); }
            bool operator==(const Point& rhs) { return x == rhs.x && y == rhs.y; }
        };

        int randInt(int a, int b) {
            static long seed = std::chrono::system_clock::now().time_since_epoch().count();
            static std::default_random_engine randEngine{seed};
            std::uniform_int_distribution dist{a, b};
            return dist(randEngine);
        }

        bool isLegal(Point p, unsigned gridSize) {
            return p.x > 0 && p.x < gridSize - 1 && p.y > 0 && p.y < gridSize - 1;
        }

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

        std::list<Point> getNeighbours(unsigned gridSize, Point p) {
            std::list<Point> newNodes;
            newNodes.push_back({p.x, p.y - 2});
            newNodes.push_back({p.x, p.y + 2});
            newNodes.push_back({p.x - 2, p.y});
            newNodes.push_back({p.x + 2, p.y});
            newNodes.remove_if([gridSize](Point p) { return !isLegal(p, gridSize); });
            newNodes.sort();
            return newNodes;
        }

        Point popPoint(std::list<Point>& points) {
            unsigned index = randInt(0, points.size() - 1);
            auto it = points.begin();
            for (int i = 0; i < index; i++) {
                it++;
            }
            Point p = *it;
            points.erase(it);
            return p;
        }

        std::vector<std::vector<bool>> generateGrid(unsigned size, float loopFactor) {
            std::vector<std::vector<bool>> grid;

            grid.resize(size);
            for (unsigned i = 0; i < size; i++) {
                grid[i].resize(size);
            }

            for (unsigned i = 0; i < size; i++) {
                for (unsigned j = 0; j < size; j++) {
                    grid[i][j] = false;
                }
            }

            Point init(randInt(0, (size - 2) / 2) * 2 + 1, randInt(0, (size - 2) / 2) * 2 + 1);

            grid[init.x][init.y] = true;
            std::list<Point> frontierPoints{getNeighbours(size, init)};
            while (!frontierPoints.empty()) {
                Point p1{popPoint(frontierPoints)};
                grid[p1.x][p1.y] = true;
                std::list<Point> neighbours{getNeighbours(size, p1)};
                neighbours.remove_if([&grid](Point p) { return !grid[p.x][p.y]; });
                Point p2{popPoint(neighbours)};
                grid[p2.x - (p2.x - p1.x) / 2][p2.y - (p2.y - p1.y) / 2] = true;
                std::list<Point> newFrontierPoints{getNeighbours(size, p1)};
                newFrontierPoints.remove_if([&grid](Point p) { return grid[p.x][p.y]; });
                frontierPoints.merge(newFrontierPoints);
                frontierPoints.unique();
            }

            const unsigned loops = size * size * loopFactor * loopFactor;
            for (unsigned i = 0; i < loops; i++) {
                Point p;
                do {
                    p = Point(randInt(1, size - 2), randInt(1, size - 2));
                } while (!isWall(grid, p));
                grid[p.x][p.y] = true;
            }

            grid[1][0] = true;
            grid[size - 2][size - 1] = true;

            return grid;
        }

        bool isHCorridor(Point p, std::vector<std::vector<bool>> const& grid) {
            return grid[p.x - 1][p.y] && grid[p.x + 1][p.y] && !grid[p.x][p.y - 1] &&
                   !grid[p.x][p.y + 1];
        }

        bool isVCorridor(Point p, std::vector<std::vector<bool>> const& grid) {
            return !grid[p.x - 1][p.y] && !grid[p.x + 1][p.y] && grid[p.x][p.y - 1] &&
                   grid[p.x][p.y + 1];
        }

        int calcCost(Point one, Point two) {
            return std::abs(one.x - two.x) + std::abs(one.y - two.y);
        }

        std::list<std::shared_ptr<Maze::Node>>
        generateGraph(std::vector<std::vector<bool>> const& grid) {
            using Node = Maze::Node;
            using Edge = Maze::Edge;
            using EdgeList = std::list<Edge>;

            const int size = grid.size();
            std::list<std::shared_ptr<Node>> graph;
            std::list<std::shared_ptr<Node>> above;

            std::shared_ptr<Node> first = std::make_shared<Node>(1, 0, EdgeList{});
            graph.push_back(first);
            above.push_back(first);

            for (int i = 1; i < size - 1; i++) {
                std::shared_ptr<Node> left = nullptr;
                for (int j = 1; j < size - 1; j++) {
                    if (grid[i][j]) {
                        if (!isHCorridor({i, j}, grid) && !isVCorridor({i, j}, grid)) {
                            EdgeList edges;
                            if (left) {
                                int cost = calcCost({i, j}, {left->x, left->y});
                                edges.push_back({left, cost});
                            }
                            auto nodeAbove =
                                std::find_if(above.begin(), above.end(), [j](auto const& nodePtr) {
                                    return nodePtr->x == j;
                                });
                            if (nodeAbove != above.end()) {
                                int cost = calcCost({i, j}, {(*nodeAbove)->x, (*nodeAbove)->y});
                                edges.push_back({*nodeAbove, cost});
                                above.erase(nodeAbove);
                            }
                            std::shared_ptr<Node> node = std::make_shared<Node>(i, j, edges);
                            graph.push_back(node);
                            above.push_back(node);
                            for (Edge const& edge : edges) {
                                edge.node->edges.push_back({node, edge.cost});
                            }
                        }
                    } else {
                        left = nullptr;
                        above.remove_if([j](auto const& nodePtr) { return nodePtr->x == j; });
                    }
                }
            }

            auto nodeAbove = std::find_if(above.begin(), above.end(), [size](auto const& nodePtr) {
                return nodePtr->x == size - 2;
            });
            int cost = calcCost({size - 2, size - 1}, {(*nodeAbove)->x, (*nodeAbove)->y});
            EdgeList edges;
            edges.push_back({*nodeAbove, cost});
            std::shared_ptr<Node> last = std::make_shared<Node>(size - 2, size - 1, edges);
            graph.push_back(last);
            for (Edge const& edge : edges) {
                edge.node->edges.push_back({last, edge.cost});
            }

            return graph;
        }
    } // namespace

    Maze::Maze(unsigned size, float loopFactor)
        : size_{size}
        , grid_{generateGrid(size, loopFactor)}
        , graph_{generateGraph(grid_)} {}

    void Maze::print() {
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

    void Maze::printGraph() {
        std::vector<std::vector<bool>> graphGrid;
        graphGrid.resize(size_);
        for (int i = 0; i < size_; i++) {
            graphGrid[i].resize(size_);
            for (int j = 0; j < size_; j++) {
                graphGrid[i][j] = false;
            }
        }

        for (std::shared_ptr<Node> nodePtr : graph_) {
            graphGrid[nodePtr->x][nodePtr->y] = true;
        }

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

    void Maze::writePng(std::string filename) {
        png::image<png::rgb_pixel> image{size_, size_};
        for (int j = 0; j < size_; j++) {
            for (int i = 0; i < size_; i++) {
                if (grid_[i][j]) {
                    image[j][i] = png::rgb_pixel(255, 255, 255);
                } else {
                    image[j][i] = png::rgb_pixel(0, 0, 0);
                }
            }
        }
        image.write(filename);
    }

    void Maze::writePngGraph(std::string filename) {
        std::vector<std::vector<bool>> graphGrid;
        graphGrid.resize(size_);
        for (int i = 0; i < size_; i++) {
            graphGrid[i].resize(size_);
            for (int j = 0; j < size_; j++) {
                graphGrid[i][j] = false;
            }
        }

        for (std::shared_ptr<Node> nodePtr : graph_) {
            graphGrid[nodePtr->x][nodePtr->y] = true;
        }

        png::image<png::rgb_pixel> image{size_, size_};
        for (int j = 0; j < size_; j++) {
            for (int i = 0; i < size_; i++) {
                if (graphGrid[i][j]) {
                    image[j][i] = png::rgb_pixel(255, 0, 0);
                } else if (grid_[i][j]) {
                    image[j][i] = png::rgb_pixel(255, 255, 255);
                } else {
                    image[j][i] = png::rgb_pixel(0, 0, 0);
                }
            }
        }
        image.write(filename);
    } // namespace mazes
} // namespace mazes
