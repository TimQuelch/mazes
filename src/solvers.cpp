#include <algorithm>
#include <deque>
#include <exception>
#include <iostream>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>

#include "maze.h"
#include "solvers.h"

namespace mazes {
    using Node = Maze::Node;
    using Edge = Maze::Edge;
    using NodePtr = std::shared_ptr<Node>;

    namespace detail {
        // Reconstruct the path
        std::list<NodePtr> reconstructPath(std::unordered_map<NodePtr, NodePtr> paths,
                                           NodePtr end) {
            std::list<NodePtr> path;
            NodePtr current = end;
            while (current) {
                path.push_back(current);
                current = paths[current];
            }
            path.reverse();
            return path;
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

    // Solve the maze using breadth first search
    std::list<NodePtr> solveBfs(Maze const& maze, NodePtr start, NodePtr end) {
        const std::list<NodePtr> graph = maze.getGraph();

        std::deque<NodePtr> queue;
        std::unordered_set<NodePtr> visited;
        std::unordered_map<NodePtr, NodePtr> paths;

        // Add first node
        queue.push_back(*(std::find_if(
            graph.begin(), graph.end(), [start](auto const& n) { return n == start; })));
        paths[start] = NodePtr{nullptr};

        while (!queue.empty()) {
            NodePtr current = queue.front();
            queue.pop_front();

            // Return if path is found
            if (current == end) {
                return detail::reconstructPath(paths, current);
            }

            // Process each of the connections
            for (Edge edge : current->edges) {
                if (!detail::contains(visited, edge.node)) {
                    if (!detail::contains(queue, edge.node)) {
                        paths[edge.node] = current;
                        queue.push_back(edge.node);
                    }
                    visited.insert(current);
                }
            }
        }
        throw std::runtime_error{"Path not found"};
    }

    // Solve the maze using depth first search
    std::list<NodePtr> solveDfs(Maze const& maze, NodePtr start, NodePtr end) {
        const std::list<NodePtr> graph = maze.getGraph();

        std::deque<NodePtr> stack;
        std::unordered_set<NodePtr> visited;
        std::unordered_map<NodePtr, NodePtr> paths;

        // Add first node
        stack.push_front(*(std::find_if(
            graph.begin(), graph.end(), [start](auto const& n) { return n == start; })));
        paths[start] = NodePtr{nullptr};

        while (!stack.empty()) {
            NodePtr current = stack.front();
            stack.pop_front();

            // Return if path is found
            if (current == end) {
                return detail::reconstructPath(paths, current);
            }

            // Process each of the connections
            for (Edge edge : current->edges) {
                if (!detail::contains(visited, edge.node)) {
                    if (!detail::contains(stack, edge.node)) {
                        paths[edge.node] = current;
                        stack.push_front(edge.node);
                    }
                    visited.insert(current);
                }
            }
        }
        throw std::runtime_error{"Path not found"};
    }

    // Solve the maze using Djikstra's algorithm
    std::list<NodePtr> solveDijkstra(Maze const& maze, NodePtr start, NodePtr end) {
        const std::list<NodePtr> graph = maze.getGraph();

        std::unordered_map<NodePtr, int> costs;
        std::unordered_map<NodePtr, NodePtr> paths;
        paths[start] = NodePtr{nullptr};

        using queueNode = std::pair<NodePtr, int>;
        auto compare = [](queueNode const& one, queueNode const& two) {
            return one.second > two.second;
        };
        std::priority_queue<queueNode, std::vector<queueNode>, decltype(compare)> queue(compare);

        for (NodePtr const& node : graph) {
            costs[node] = std::numeric_limits<int>::max();
        }
        costs[start] = 0;
        queue.push(std::make_pair(start, costs[start]));

        while (!queue.empty()) {
            NodePtr current;
            int poppedCost;
            std::tie(current, poppedCost) = queue.top();
            queue.pop();
            if (poppedCost == costs[current]) {
                if (current == end) {
                    return detail::reconstructPath(paths, current);
                }

                int currentCost = costs[current];
                for (Edge edge : current->edges) {
                    int newCost = currentCost + edge.cost;
                    if (newCost < costs[edge.node]) {
                        costs[edge.node] = newCost;
                        paths[edge.node] = current;
                        queue.push(std::make_pair(edge.node, newCost));
                    }
                }
            }
        }
        throw std::runtime_error{"Path not found"};
    }

    // Solve the maze using A*
    std::list<NodePtr> solveAstar(Maze const& maze, NodePtr start, NodePtr end) {
        const std::list<NodePtr> graph = maze.getGraph();

        std::unordered_map<NodePtr, int> costs;
        std::unordered_map<NodePtr, int> distances;
        std::unordered_map<NodePtr, NodePtr> paths;
        paths[start] = NodePtr{nullptr};

        using queueNode = std::pair<NodePtr, int>;
        auto compare = [](queueNode const& one, queueNode const& two) {
            return one.second > two.second;
        };
        std::priority_queue<queueNode, std::vector<queueNode>, decltype(compare)> queue(compare);

        for (NodePtr const& node : graph) {
            costs[node] = std::numeric_limits<int>::max();
            distances[node] = std::abs(end->x - node->x) + std::abs(end->y - node->y);
        }
        costs[start] = 0;
        queue.push(std::make_pair(start, costs[start] + distances[start]));

        while (!queue.empty()) {
            NodePtr current;
            int poppedCost;
            std::tie(current, poppedCost) = queue.top();
            queue.pop();
            if (poppedCost == costs[current] + distances[current]) {
                if (current == end) {
                    return detail::reconstructPath(paths, current);
                }

                int currentCost = costs[current];
                for (Edge edge : current->edges) {
                    int newCost = currentCost + edge.cost;
                    if (newCost < costs[edge.node]) {
                        costs[edge.node] = newCost;
                        paths[edge.node] = current;
                        queue.push(std::make_pair(edge.node, newCost + distances[edge.node]));
                    }
                }
            }
        }
        throw std::runtime_error{"Path not found"};
        return {};
    }

} // namespace mazes
