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
            int x;
            int y;
            std::list<Edge> edges;

            Node(int x, int y, std::list<Edge> edges)
                : x{x}
                , y{y}
                , edges{std::move(edges)} {}
        };

        struct Edge {
        public:
            std::shared_ptr<Node> node;
            int cost;

            Edge(std::shared_ptr<Node> node, int cost)
                : node{node}
                , cost{cost} {}
        };

        Maze(unsigned size = DEFAULT_MAZE_SIZE, float loopFactor = DEFAULT_LOOP_FACTOR);

        std::list<std::shared_ptr<Node>> const& getGraph() const { return graph_; }

        void print() const;
        void printGraph() const;

        void writePng(std::string filename) const;
        void writePngGraph(std::string filename) const;

    private:
        unsigned size_;
        std::vector<std::vector<bool>> grid_;
        std::list<std::shared_ptr<Node>> graph_;
    };
} // namespace mazes
