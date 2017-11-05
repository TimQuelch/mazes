#include <chrono>
#include <iomanip>
#include <iostream>
#include <list>
#include <random>
#include <tuple>

#include "maze.h"

namespace mazes {
    namespace {
        constexpr float LOOP_FACTOR = 0.1;

        struct Point {
            unsigned x;
            unsigned y;

            Point(unsigned x = 0, unsigned y = 0)
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
            bool wall = !grid[p.x][p.y];
            bool nsWall = !grid[p.x][p.y + 1] && !grid[p.x][p.y - 1];
            bool esWall = !grid[p.x - 1][p.y] && !grid[p.x + 1][p.y];
            return wall && ((nsWall || esWall) && !(nsWall && esWall));
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
    } // namespace

    Maze::Maze(unsigned size)
        : size_{size} {
        grid_.resize(size_);
        for (unsigned i = 0; i < size_; i++) {
            grid_[i].resize(size_);
        }

        for (unsigned i = 0; i < size_; i++) {
            for (unsigned j = 0; j < size_; j++) {
                grid_[i][j] = false;
            }
        }

        Point init(randInt(0, (size_ - 2) / 2) * 2 + 1, randInt(0, (size_ - 2) / 2) * 2 + 1);

        grid_[init.x][init.y] = true;
        std::list<Point> frontierPoints{getNeighbours(size_, init)};
        while (!frontierPoints.empty()) {
            Point p1{popPoint(frontierPoints)};
            grid_[p1.x][p1.y] = true;
            std::list<Point> neighbours{getNeighbours(size, p1)};
            neighbours.remove_if([this](Point p) { return !grid_[p.x][p.y]; });
            Point p2{popPoint(neighbours)};
            grid_[p2.x - (static_cast<int>(p2.x) - static_cast<int>(p1.x)) / 2]
                 [p2.y - (static_cast<int>(p2.y) - static_cast<int>(p1.y)) / 2] = true;
            std::list<Point> newFrontierPoints{getNeighbours(size_, p1)};
            newFrontierPoints.remove_if([this](Point p) { return grid_[p.x][p.y]; });
            frontierPoints.merge(newFrontierPoints);
            frontierPoints.unique();
        }

        const unsigned loops = size_ * size_ * LOOP_FACTOR * LOOP_FACTOR;
        for (unsigned i = 0; i < loops; i++) {
            Point p;
            do {
                p = Point(randInt(1, size_ - 2), randInt(1, size_ - 2));
            } while (!isWall(grid_, p));
            grid_[p.x][p.y] = true;
        }

        grid_[0][1] = true;
        grid_[size_ - 1][size_ - 2] = true;
    }

    void Maze::print() {
        std::cout << "    ";
        for (int i = 0; i < size_; i++) {
            std::cout << std::setw(2) << i << " ";
        }
        std::cout << "\n\n";
        for (int i = 0; i < size_; i++) {
            std::cout << std::setw(2) << i << "  ";
            for (int j = 0; j < size_; j++) {
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
} // namespace mazes
