#include <algorithm>
#include <deque>
#include <iostream>
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
                if (std::find(visited.begin(), visited.end(), edge.node) == visited.end()) {
                    if (std::find(queue.begin(), queue.end(), edge.node) == queue.end()) {
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
                if (std::find(visited.begin(), visited.end(), edge.node) == visited.end()) {
                    if (std::find(stack.begin(), stack.end(), edge.node) == stack.end()) {
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
        return graph;
    }

    std::list<NodePtr> solveAstar(Maze const& maze, NodePtr start, NodePtr end) {
        const std::list<NodePtr> graph = maze.getGraph();
        return graph;
    }

} // namespace mazes
