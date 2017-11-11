#include <algorithm>
#include <deque>
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

    namespace {
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

        //template <>
        template <typename T>
        bool contains(std::unordered_set<T> container, T const& value) {
            return container.find(value) != container.end();
        }
    } // namespace

    std::list<NodePtr> solveBfs(Maze const& maze, NodePtr start, NodePtr end) {
        const std::list<NodePtr> graph = maze.getGraph();

        std::deque<NodePtr> queue;
        std::unordered_set<NodePtr> visited;
        std::unordered_map<NodePtr, NodePtr> paths;

        queue.push_back(*(std::find_if(
            graph.begin(), graph.end(), [start](auto const& n) { return n == start; })));
        paths[start] = NodePtr{nullptr};

        while (!queue.empty()) {
            NodePtr current = queue.front();
            queue.pop_front();
            if (current == end) {
                return reconstructPath(paths, current);
            }

            for (Edge edge : current->edges) {
                if (!contains(visited, edge.node)) {
                    if (!contains(queue, edge.node)) {
                        paths[edge.node] = current;
                        queue.push_back(edge.node);
                    }
                    visited.insert(current);
                }
            }
        }
        std::cout << "Path not found\n";
        return graph; // TODO shouldn't happen
    }

    std::list<NodePtr> solveDfs(Maze const& maze, NodePtr start, NodePtr end) {
        const std::list<NodePtr> graph = maze.getGraph();

        std::deque<NodePtr> stack;
        std::unordered_set<NodePtr> visited;
        std::unordered_map<NodePtr, NodePtr> paths;

        stack.push_front(*(std::find_if(
            graph.begin(), graph.end(), [start](auto const& n) { return n == start; })));
        paths[start] = NodePtr{nullptr};

        while (!stack.empty()) {
            NodePtr current = stack.front();
            stack.pop_front();
            if (current == end) {
                return reconstructPath(paths, current);
            }

            for (Edge edge : current->edges) {
                if (!contains(visited, edge.node)) {
                    if (!contains(stack, edge.node)) {
                        paths[edge.node] = current;
                        stack.push_front(edge.node);
                    }
                    visited.insert(current);
                }
            }
        }
        std::cout << "Path not found\n";
        return graph;
    }

    std::list<NodePtr> solveDijkstra(Maze const& maze, NodePtr start, NodePtr end) {
        const std::list<NodePtr> graph = maze.getGraph();

        std::unordered_map<NodePtr, int> costs;
        std::unordered_map<NodePtr, NodePtr> paths;
        paths[start] = NodePtr{nullptr};

        auto compare = [&costs](auto const& one, auto const& two) {
            return costs[one] > costs[two];
        };
        // std::priority_queue<NodePtr, std::vector<NodePtr>, decltype(compare)> queue(compare);
        std::vector<NodePtr> queue;

        for (NodePtr const& node : graph) {
            costs[node] = std::numeric_limits<int>::max();
            queue.push_back(node);
        }
        costs[start] = 0;
        std::make_heap(queue.begin(), queue.end(), compare);

        while (!queue.empty()) {
            NodePtr current = queue.front();
            std::pop_heap(queue.begin(), queue.end(), compare);
            queue.pop_back();
            if (current == end) {
                return reconstructPath(paths, current);
            }

            int currentCost = costs[current];
            for (Edge edge : current->edges) {
                int newCost = currentCost + edge.cost;
                if (newCost < costs[edge.node]) {
                    costs[edge.node] = newCost;
                    paths[edge.node] = current;
                    std::make_heap(queue.begin(), queue.end(), compare);
                }
            }
        }

        return graph;
    }

    std::list<NodePtr> solveAstar(Maze const& maze, NodePtr start, NodePtr end) {
        const std::list<NodePtr> graph = maze.getGraph();
        return graph;
    }

} // namespace mazes
