/// \author Tim Quelch

#include <algorithm>
#include <exception>
#include <iostream>
#include <limits>
#include <queue>
#include <stack>
#include <unordered_map>
#include <unordered_set>

#include "maze.h"
#include "solvers.h"

namespace mazes {
    /// \cond
    using Node = Maze::Node;
    using Edge = Maze::Edge;
    using NodePtr = std::shared_ptr<Node>;
    /// \endcond

    namespace detail {
        /// Reconstruct the path from the parent map and end point
        /// \param paths Map of parent nodes. The value for each key is the node that preceded the key in the path
        /// \param end The end node
        /// \returns The list of nodes in the path, in order
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

        /// Helper to test if container contains a value
        /// \returns true if the container contains the value
        template <typename Container>
        bool contains(Container container, typename Container::value_type const& value) {
            return std::find(container.begin(), container.end(), value) != container.end();
        }

        /// Checks if a unordered_set contains a value. Specialised overload, as this is much more
        /// efficient that std::find
        /// \returns true if the container contains the value
        template <typename T>
        bool contains(std::unordered_set<T> container, T const& value) {
            return container.find(value) != container.end();
        }
    } // namespace detail

    /// \cond

    // Solve the maze using breadth first search
    std::list<NodePtr> solveBfs(Maze const& maze, NodePtr start, NodePtr end) {
        return solveBfs(maze, start, end, "");
    }

    std::list<std::shared_ptr<Maze::Node>> solveBfs(Maze const& maze,
                                                    std::shared_ptr<Maze::Node> start,
                                                    std::shared_ptr<Maze::Node> end,
                                                    std::string_view filename) {
        const std::list<NodePtr> graph = maze.getGraph();
        VideoWriter video{maze, filename};
        const bool writeVideo{!filename.empty()};

        std::queue<NodePtr> queue;
        std::unordered_set<NodePtr> visited;
        std::unordered_map<NodePtr, NodePtr> paths;

        // Add first node
        queue.push(*(std::find_if(
            graph.begin(), graph.end(), [start](auto const& n) { return n == start; })));
        paths[start] = NodePtr{nullptr};

        while (!queue.empty()) {
            NodePtr current = queue.front();
            queue.pop();

            if (writeVideo) {
                video.updateTile(current->x, current->y, Tile::visited);
            }

            if (!detail::contains(visited, current)) {
                // Return if path is found
                if (current == end) {
                    return detail::reconstructPath(paths, current);
                }

                // Process each of the connections
                for (Edge edge : current->edges) {
                    if (!detail::contains(visited, edge.node)) {
                        paths[edge.node] = current;
                        queue.push(edge.node);
                        visited.insert(current);
                        if (writeVideo) {
                            video.updateTile(edge.node->x, edge.node->y, Tile::discovered);
                        }
                    }
                }
            }
            video.writeFrame();
        }
        throw std::runtime_error{"Path not found"};
    }

    // Solve the maze using depth first search
    std::list<NodePtr> solveDfs(Maze const& maze, NodePtr start, NodePtr end) {
        return solveDfs(maze, start, end, "");
    }

    // Solve the maze using depth first search
    std::list<NodePtr>
    solveDfs(Maze const& maze, NodePtr start, NodePtr end, std::string_view filename) {
        const std::list<NodePtr> graph = maze.getGraph();

        VideoWriter video{maze, filename};
        bool writeVideo{!filename.empty()};

        std::stack<NodePtr> stack;
        std::unordered_set<NodePtr> visited;
        std::unordered_map<NodePtr, NodePtr> paths;

        // Add first node
        stack.push(*(std::find_if(
            graph.begin(), graph.end(), [start](auto const& n) { return n == start; })));
        paths[start] = NodePtr{nullptr};

        while (!stack.empty()) {
            NodePtr current = stack.top();
            stack.pop();

            if (writeVideo) {
                video.updateTile(current->x, current->y, Tile::visited);
            }

            if (!detail::contains(visited, current)) {
                // Return if path is found
                if (current == end) {
                    return detail::reconstructPath(paths, current);
                }

                // Process each of the connections
                for (Edge edge : current->edges) {
                    if (!detail::contains(visited, edge.node)) {
                        paths[edge.node] = current;
                        stack.push(edge.node);
                        visited.insert(current);
                        if (writeVideo) {
                            video.updateTile(edge.node->x, edge.node->y, Tile::discovered);
                        }
                    }
                }
            }
            video.writeFrame();
        }
        throw std::runtime_error{"Path not found"};
    }

    // Solve the maze using Djikstra's algorithm
    std::list<NodePtr> solveDijkstra(Maze const& maze, NodePtr start, NodePtr end) {
        return solveDijkstra(maze, start, end, "");
    }

    // Solve the maze using Djikstra's algorithm
    std::list<NodePtr>
    solveDijkstra(Maze const& maze, NodePtr start, NodePtr end, std::string_view filename) {
        const std::list<NodePtr> graph = maze.getGraph();

        VideoWriter video{maze, filename};
        bool writeVideo{!filename.empty()};

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

            if (writeVideo) {
                video.updateTile(current->x, current->y, Tile::visited);
            }

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

                        if (writeVideo) {
                            video.updateTile(edge.node->x, edge.node->y, Tile::discovered);
                        }
                    }
                }
            }
            video.writeFrame();
        }
        throw std::runtime_error{"Path not found"};
    }

    // Solve the maze using A*
    std::list<NodePtr> solveAstar(Maze const& maze, NodePtr start, NodePtr end) {
        return solveAstar(maze, start, end, "");
    }

    // Solve the maze using A*
    std::list<NodePtr>
    solveAstar(Maze const& maze, NodePtr start, NodePtr end, std::string_view filename) {
        const std::list<NodePtr> graph = maze.getGraph();

        VideoWriter video{maze, filename};
        bool writeVideo{!filename.empty()};

        std::unordered_map<NodePtr, int> costs;
        std::unordered_map<NodePtr, NodePtr> paths;
        paths[start] = NodePtr{nullptr};

        auto distance = [&end](NodePtr const& node) -> int {
            static const int ex = end->x;
            static const int ey = end->y;
            return ex - node->x + ey - node->y;
        };

        using queueNode = std::pair<NodePtr, int>;
        auto compare = [](queueNode const& one, queueNode const& two) {
            return one.second > two.second;
        };
        std::priority_queue<queueNode, std::vector<queueNode>, decltype(compare)> queue(compare);

        for (NodePtr const& node : graph) {
            costs[node] = std::numeric_limits<int>::max();
        }
        costs[start] = 0;
        queue.push(std::make_pair(start, costs[start] + distance(start)));

        while (!queue.empty()) {
            NodePtr current;
            int poppedCost;
            std::tie(current, poppedCost) = queue.top();
            queue.pop();

            if (writeVideo) {
                video.updateTile(current->x, current->y, Tile::visited);
            }

            if (poppedCost == costs[current] + distance(current)) {
                if (current == end) {
                    return detail::reconstructPath(paths, current);
                }

                int currentCost = costs[current];
                for (Edge edge : current->edges) {
                    int newCost = currentCost + edge.cost;
                    if (newCost < costs[edge.node]) {
                        costs[edge.node] = newCost;
                        paths[edge.node] = current;
                        queue.push(std::make_pair(edge.node, newCost + distance(edge.node)));
                        if (writeVideo) {
                            video.updateTile(edge.node->x, edge.node->y, Tile::discovered);
                        }
                    }
                }
            }
            video.writeFrame();
        }
        throw std::runtime_error{"Path not found"};
    }
    /// \endcond
} // namespace mazes
