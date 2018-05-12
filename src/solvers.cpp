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
        reconstructPath(std::unordered_map<const Maze::Node*, const Maze::Node*> paths,
                        const Maze::Node* end,
                        const std::list<std::shared_ptr<Maze::Node>>& graph) {
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

        void writePath(VideoWriter& video, const std::list<std::shared_ptr<Maze::Node>>& path) {
            auto prev = path.cbegin();
            auto next = prev++;
            while (prev != path.cend()) {
                video.updateLine((*prev)->x, (*prev)->y, (*next)->x, (*next)->y, Tile::path);
                video.writeUpdate();
                prev++;
                next++;
            }
        }

        template <typename Container>
        bool contains(Container container, const typename Container::value_type& value) {
            return std::find(container.begin(), container.end(), value) != container.end();
        }

        template <typename T>
        bool contains(std::unordered_set<T> container, const T& value) {
            return container.find(value) != container.end();
        }
    } // namespace detail

    std::list<std::shared_ptr<Maze::Node>> solveBfs(const Maze& maze,
                                                    const Maze::Node& start,
                                                    const Maze::Node& end,
                                                    std::optional<VideoWriter>& video) {
        const auto graph = maze.getGraph();

        auto queue = std::queue<const Maze::Node*>{};
        auto visited = std::unordered_set<const Maze::Node*>{};
        auto paths = std::unordered_map<const Maze::Node*, const Maze::Node*>{};

        // Add first node
        queue.push(&start);
        paths[&start] = nullptr;

        while (!queue.empty()) {
            const auto current = queue.front();
            queue.pop();

            if (video) {
                const auto prev = paths[current];
                if (prev) {
                    video->updateLine(prev->x, prev->y, current->x, current->y, Tile::visited);
                }
            }

            if (!detail::contains(visited, current)) {
                // Return if path is found
                if (current == &end) {
                    const auto path = detail::reconstructPath(paths, current, graph);
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
                                              Tile::discovered);
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
    std::list<std::shared_ptr<Maze::Node>> solveDfs(const Maze& maze,
                                                    const Maze::Node& start,
                                                    const Maze::Node& end,
                                                    std::optional<VideoWriter>& video) {
        const auto graph = maze.getGraph();

        auto stack = std::stack<const Maze::Node*>{};
        auto visited = std::unordered_set<const Maze::Node*>{};
        auto paths = std::unordered_map<const Maze::Node*, const Maze::Node*>{};

        // Add first node
        stack.push(&start);
        paths[&start] = nullptr;

        while (!stack.empty()) {
            const auto current = stack.top();
            stack.pop();

            if (video) {
                const auto prev = paths[current];
                if (prev) {
                    video->updateLine(prev->x, prev->y, current->x, current->y, Tile::visited);
                }
            }

            if (!detail::contains(visited, current)) {
                // Return if path is found
                if (&(*current) == &end) {
                    const auto path = detail::reconstructPath(paths, current, graph);
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
                                              Tile::discovered);
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
    std::list<std::shared_ptr<Maze::Node>> solveDijkstra(const Maze& maze,
                                                         const Maze::Node& start,
                                                         const Maze::Node& end,
                                                         std::optional<VideoWriter>& video) {
        const auto graph = maze.getGraph();

        auto costs = std::unordered_map<const Maze::Node*, int>{};
        auto paths = std::unordered_map<const Maze::Node*, const Maze::Node*>{};
        paths[&start] = nullptr;

        using CostNode = std::pair<const Maze::Node*, int>;
        auto cmp = [](const auto& one, const auto& two) { return one.second > two.second; };
        auto queue = std::priority_queue<CostNode, std::vector<CostNode>, decltype(cmp)>{cmp};

        for (const auto& node : graph) {
            costs[node.get()] = std::numeric_limits<int>::max();
        }
        costs[&start] = 0;
        queue.push(std::make_pair(&start, costs[&start]));

        while (!queue.empty()) {
            auto [current, poppedCost] = queue.top();
            queue.pop();

            if (video) {
                const auto prev = paths[current];
                if (prev) {
                    video->updateLine(prev->x, prev->y, current->x, current->y, Tile::visited);
                }
            }

            if (poppedCost == costs[current]) {
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
                        queue.push(std::make_pair(edge.node.get(), newCost));

                        if (video) {
                            video->updateLine(current->x,
                                              current->y,
                                              edge.node->x,
                                              edge.node->y,
                                              Tile::discovered);
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
    std::list<std::shared_ptr<Maze::Node>> solveAstar(const Maze& maze,
                                                      const Maze::Node& start,
                                                      const Maze::Node& end,
                                                      std::optional<VideoWriter>& video,
                                                      double heuristicWeighting) {
        const auto graph = maze.getGraph();

        auto costs = std::unordered_map<const Maze::Node*, int>{};
        auto paths = std::unordered_map<const Maze::Node*, const Maze::Node*>{};
        paths[&start] = nullptr;

        auto distance = [&end, heuristicWeighting](const Maze::Node* node) -> int {
            static const int ex = end.x;
            static const int ey = end.y;
            return heuristicWeighting * std::ceil(std::sqrt((ex - node->x) * (ex - node->x) +
                                                            (ey - node->y) * (ey - node->y)));
        };

        using CostNode = std::pair<const Maze::Node*, int>;
        auto cmp = [](const CostNode& one, const CostNode& two) { return one.second > two.second; };
        auto queue = std::priority_queue<CostNode, std::vector<CostNode>, decltype(cmp)>{cmp};

        for (const auto& node : graph) {
            costs[node.get()] = std::numeric_limits<int>::max();
        }
        costs[&start] = 0;
        queue.push(std::make_pair(&start, costs[&start] + distance(&start)));

        while (!queue.empty()) {
            const Maze::Node* current;
            int poppedCost;
            std::tie(current, poppedCost) = queue.top();
            queue.pop();

            if (video) {
                const auto prev = paths[current];
                if (prev) {
                    video->updateLine(prev->x, prev->y, current->x, current->y, Tile::visited);
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
                                              Tile::discovered);
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
