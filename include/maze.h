#ifndef MAZES_MAZE_H
#define MAZES_MAZE_H

#include <list>
#include <memory>
#include <vector>

namespace mazes {
    constexpr unsigned DEFAULT_MAZE_SIZE = 21;
    constexpr float DEFAULT_LOOP_FACTOR = 0;

    class Maze {
    public:
        struct Edge;
        struct Node {
        public:
            const int x;
            const int y;
            std::list<Edge> edges;

            Node(int x, int y, std::list<Edge> edges)
                : x{x}
                , y{y}
                , edges{std::move(edges)} {}
        };

        struct Edge {
        public:
            const std::shared_ptr<Node> node;
            const int cost;

            Edge(std::shared_ptr<Node> node, int cost)
                : node{node}
                , cost{cost} {}
        };

        Maze(unsigned size = DEFAULT_MAZE_SIZE, float loopFactor = DEFAULT_LOOP_FACTOR);

        std::list<std::shared_ptr<Node>> const& getGraph() const { return graph_; }
        std::shared_ptr<Node> getStartNode() const { return graph_.front(); }
        std::shared_ptr<Node> getEndNode() const { return graph_.back(); }

        void print() const;
        void printGraph() const;

        void writePng(std::string_view filename) const;
        void writePngGraph(std::string_view filename) const;

        void writePngPath(std::list<std::shared_ptr<Node>> path, std::string_view filename) const;

    private:
        unsigned size_;
        std::vector<std::vector<bool>> grid_;
        std::list<std::shared_ptr<Node>> graph_;
    };
} // namespace mazes

#endif
