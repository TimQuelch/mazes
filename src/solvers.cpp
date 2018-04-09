/// \author Tim Quelch

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <iostream>
#include <limits>
#include <queue>
#include <stack>
#include <unordered_map>
#include <unordered_set>

#include "maze.h"
#include "solvers.h"
#include "video-writer.h"

using namespace std::chrono_literals;
constexpr auto freezeDuration = 2000ms;

namespace mazes {
    namespace detail {
        std::list<std::shared_ptr<Maze::Node>>
        reconstructPath(std::unordered_map<Maze::Node const*, Maze::Node const*> paths,
                        Maze::Node const* end,
                        std::list<std::shared_ptr<Maze::Node>> const& graph) {
            auto path = std::list<std::shared_ptr<Maze::Node>>{};
            auto current = end;
            while (current) {
                auto inGraph = std::find_if(
                    graph.begin(), graph.end(), [current](auto n) { return n.get() == current; });
                path.push_back(*inGraph);
                current = paths[current];
            }
            path.reverse();
            return path;
        }

        void writePath(VideoWriter& video, std::list<std::shared_ptr<Maze::Node>> const& path) {
            auto prev = path.cbegin();
            auto next = prev++;
            while (prev != path.cend()) {
                video.updateLine(
                    (*prev)->x, (*prev)->y, (*next)->x, (*next)->y, VideoWriter::Tile::path);
                video.writeUpdate();
                prev++;
                next++;
            }
        }

        template <typename Container>
        bool contains(Container container, typename Container::value_type const& value) {
            return std::find(container.begin(), container.end(), value) != container.end();
        }

        template <typename T>
        bool contains(std::unordered_set<T> container, T const& value) {
            return container.find(value) != container.end();
        }
    } // namespace detail

    std::list<std::shared_ptr<Maze::Node>> solveBfs(Maze const& maze,
                                                    Maze::Node const& start,
                                                    Maze::Node const& end,
                                                    std::optional<VideoWriter>& video) {
        auto const graph = maze.getGraph();

        auto queue = std::queue<Maze::Node const*>{};
        auto visited = std::unordered_set<Maze::Node const*>{};
        auto paths = std::unordered_map<Maze::Node const*, Maze::Node const*>{};

        // Add first node
        queue.push(&start);
        paths[&start] = nullptr;

        while (!queue.empty()) {
            auto const current = queue.front();
            queue.pop();

            if (video) {
                auto const prev = paths[current];
                if (prev) {
                    video->updateLine(
                        prev->x, prev->y, current->x, current->y, VideoWriter::Tile::visited);
                }
            }

            if (!detail::contains(visited, current)) {
                // Return if path is found
                if (current == &end) {
                    auto const path = detail::reconstructPath(paths, current, graph);
                    if (video) {
                        detail::writePath(*video, path);
                        video->writeFreezeFrame(freezeDuration);
                    }
                    return path;
                }

                // Process each of the connections
                for (auto& edge : current->edges) {
                    if (!detail::contains(visited, edge.node.get())) {
                        paths[edge.node.get()] = current;
                        queue.push(edge.node.get());
                        visited.insert(current);
                        if (video) {
                            video->updateLine(current->x,
                                              current->y,
                                              edge.node->x,
                                              edge.node->y,
                                              VideoWriter::Tile::discovered);
                        }
                    }
                }
            }
            if (video) {
                video->writeUpdate();
            }
        }
        throw std::runtime_error{"Path not found"};
    }

    // Solve the maze using depth first search
    std::list<std::shared_ptr<Maze::Node>> solveDfs(Maze const& maze,
                                                    Maze::Node const& start,
                                                    Maze::Node const& end,
                                                    std::optional<VideoWriter>& video) {
        auto const graph = maze.getGraph();

        auto stack = std::stack<Maze::Node const*>{};
        auto visited = std::unordered_set<Maze::Node const*>{};
        auto paths = std::unordered_map<Maze::Node const*, Maze::Node const*>{};

        // Add first node
        stack.push(&start);
        paths[&start] = nullptr;

        while (!stack.empty()) {
            auto const current = stack.top();
            stack.pop();

            if (video) {
                auto const prev = paths[current];
                if (prev) {
                    video->updateLine(
                        prev->x, prev->y, current->x, current->y, VideoWriter::Tile::visited);
                }
            }

            if (!detail::contains(visited, current)) {
                // Return if path is found
                if (&(*current) == &end) {
                    auto const path = detail::reconstructPath(paths, current, graph);
                    if (video) {
                        detail::writePath(*video, path);
                        video->writeFreezeFrame(freezeDuration);
                    }
                    return path;
                }

                // Process each of the connections
                for (auto& edge : current->edges) {
                    if (!detail::contains(visited, edge.node.get())) {
                        paths[edge.node.get()] = current;
                        stack.push(edge.node.get());
                        visited.insert(current);
                        if (video) {
                            video->updateLine(current->x,
                                              current->y,
                                              edge.node->x,
                                              edge.node->y,
                                              VideoWriter::Tile::discovered);
                        }
                    }
                }
            }
            if (video) {
                video->writeUpdate();
            }
        }
        throw std::runtime_error{"Path not found"};
    }

    // Solve the maze using Djikstra's algorithm
    std::list<std::shared_ptr<Maze::Node>> solveDijkstra(Maze const& maze,
                                                         Maze::Node const& start,
                                                         Maze::Node const& end,
                                                         std::optional<VideoWriter>& video) {
        const auto graph = maze.getGraph();

        auto costs = std::unordered_map<Maze::Node const*, int>{};
        auto paths = std::unordered_map<Maze::Node const*, Maze::Node const*>{};
        paths[&start] = nullptr;

        using CostNode = std::pair<Maze::Node const*, int>;
        auto cmp = [](auto const& one, auto const& two) { return one.second > two.second; };
        auto queue = std::priority_queue<CostNode, std::vector<CostNode>, decltype(cmp)>{cmp};

        for (auto const& node : graph) {
            costs[node.get()] = std::numeric_limits<int>::max();
        }
        costs[&start] = 0;
        queue.push(std::make_pair(&start, costs[&start]));

        while (!queue.empty()) {
            auto [current, poppedCost] = queue.top();
            queue.pop();

            if (video) {
                auto const prev = paths[current];
                if (prev) {
                    video->updateLine(
                        prev->x, prev->y, current->x, current->y, VideoWriter::Tile::visited);
                }
            }

            if (poppedCost == costs[current]) {
                if (current == &end) {
                    auto const path = detail::reconstructPath(paths, current, graph);
                    if (video) {
                        detail::writePath(*video, path);
                        video->writeFreezeFrame(freezeDuration);
                    }
                    return path;
                }

                int currentCost = costs[current];
                for (auto& edge : current->edges) {
                    int newCost = currentCost + edge.cost;
                    if (newCost < costs[edge.node.get()]) {
                        costs[edge.node.get()] = newCost;
                        paths[edge.node.get()] = current;
                        queue.push(std::make_pair(edge.node.get(), newCost));

                        if (video) {
                            video->updateLine(current->x,
                                              current->y,
                                              edge.node->x,
                                              edge.node->y,
                                              VideoWriter::Tile::discovered);
                        }
                    }
                }
            }
            if (video) {
                video->writeUpdate();
            }
        }
        throw std::runtime_error{"Path not found"};
    }

    // Solve the maze using A*
    std::list<std::shared_ptr<Maze::Node>> solveAstar(Maze const& maze,
                                                      Maze::Node const& start,
                                                      Maze::Node const& end,
                                                      std::optional<VideoWriter>& video,
                                                      double heuristicWeighting) {
        auto const graph = maze.getGraph();

        auto costs = std::unordered_map<Maze::Node const*, int>{};
        auto paths = std::unordered_map<Maze::Node const*, Maze::Node const*>{};
        paths[&start] = nullptr;

        auto distance = [&end, heuristicWeighting](Maze::Node const* node) -> int {
            static const int ex = end.x;
            static const int ey = end.y;
            return heuristicWeighting * std::ceil(std::sqrt((ex - node->x) * (ex - node->x) +
                                                            (ey - node->y) * (ey - node->y)));
        };

        using CostNode = std::pair<Maze::Node const*, int>;
        auto cmp = [](CostNode const& one, CostNode const& two) { return one.second > two.second; };
        auto queue = std::priority_queue<CostNode, std::vector<CostNode>, decltype(cmp)>{cmp};

        for (auto const& node : graph) {
            costs[node.get()] = std::numeric_limits<int>::max();
        }
        costs[&start] = 0;
        queue.push(std::make_pair(&start, costs[&start] + distance(&start)));

        while (!queue.empty()) {
            Maze::Node const* current;
            int poppedCost;
            std::tie(current, poppedCost) = queue.top();
            queue.pop();

            if (video) {
                auto const prev = paths[current];
                if (prev) {
                    video->updateLine(
                        prev->x, prev->y, current->x, current->y, VideoWriter::Tile::visited);
                }
            }

            if (poppedCost == costs[current] + distance(current)) {
                if (current == &end) {
                    const auto path = detail::reconstructPath(paths, current, graph);
                    if (video) {
                        detail::writePath(*video, path);
                        video->writeFreezeFrame(freezeDuration);
                    }
                    return path;
                }

                int currentCost = costs[current];
                for (auto& edge : current->edges) {
                    int newCost = currentCost + edge.cost;
                    if (newCost < costs[edge.node.get()]) {
                        costs[edge.node.get()] = newCost;
                        paths[edge.node.get()] = current;
                        queue.push(
                            std::make_pair(edge.node.get(), newCost + distance(edge.node.get())));
                        if (video) {
                            video->updateLine(current->x,
                                              current->y,
                                              edge.node->x,
                                              edge.node->y,
                                              VideoWriter::Tile::discovered);
                        }
                    }
                }
            }
            if (video) {
                video->writeUpdate();
            }
        }
        throw std::runtime_error{"Path not found"};
    }
} // namespace mazes
