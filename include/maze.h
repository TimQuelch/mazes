/// \author Tim Quelch

#ifndef MAZES_MAZE_H
#define MAZES_MAZE_H

#include <list>
#include <memory>
#include <vector>

namespace mazes {
    /// Default length of the side of the maze. Mazes are always square
    constexpr unsigned DEFAULT_MAZE_SIZE = 21;
    /// Default loop factor
    constexpr float DEFAULT_LOOP_FACTOR = 0;

    /// Generated maze to be solved. Consists of both a grid and a graph of intersection
    /// nodes. Enables writing to stdout and PNG image
    class Maze {
    public:
        struct Edge;

        /// A Node in graph of the maze
        struct Node {
        public:
            const int x;           ///< x coordinate of the node
            const int y;           ///< y coordinate of the node
            std::list<Edge> edges; ///< All the edges in this node

            /// Construct a Node with given values
            Node(int x, int y, std::list<Edge> edges)
                : x{x}
                , y{y}
                , edges{std::move(edges)} {}
        };

        /// An Edge from one Node to another
        struct Edge {
        public:
            const std::shared_ptr<Node> node; ///< The Node this edge leads to
            const int cost;                   ///< The cost of the edge (distance)

            /// Construct an Edge with given values
            Edge(std::shared_ptr<Node> node, int cost)
                : node{node}
                , cost{cost} {}
        };

        /// Construct a Maze with given values. Mazes are always square
        /// \param size Length of the side of the maze. This should be an odd number for good
        /// results \param loopFactor Specifies the amount of loops created in the maze. 0 means
        /// there is only one solution to the maze. Increasing it increases the number of possible
        /// solutions
        Maze(unsigned size = DEFAULT_MAZE_SIZE, float loopFactor = DEFAULT_LOOP_FACTOR);

        /// Gets a list of all the Nodes in the maze graph
        /// \returns A list of all the Nodes in the maze graph
        std::list<std::shared_ptr<Node>> const& getGraph() const { return graph_; }

        /// Gets the start Node. At this stage this is the first in the sorted list of Nodes, which,
        /// when sorted by coordinate is the top left node.
        /// \returns A pointer to the start node
        std::shared_ptr<Node> getStartNode() const { return graph_.front(); }

        /// Gets the end Node. At this stage this is the last in the sorted list of Nodes, which,
        /// when sorted by coordinate is the bottom left node.
        /// \returns A pointer to the end node
        std::shared_ptr<Node> getEndNode() const { return graph_.back(); }

        void print() const;      ///< Print the maze to stdout
        void printGraph() const; ///< Print the maze to stdout with graph intersections marked

        /// Write thet maze to a PNG file
        /// \param filename Name of the file to write the PNG image to
        void writePng(std::string_view filename) const;

        /// Write the maze to a PNG file with graph nodes marked
        /// \param filename Name of the file to write the PNG image to
        void writePngGraph(std::string_view filename) const;

        /// Write the maze to a PNG file with a path (usually a solution) marked
        /// \param path A list of the Nodes that make up the path
        /// \param filename Name of the file to write the PNG image to
        void writePngPath(std::list<std::shared_ptr<Node>> path, std::string_view filename) const;

    private:
        unsigned size_;                          ///< Length of the side of the maze
        std::vector<std::vector<bool>> grid_;    ///< Grid of the maze
        std::list<std::shared_ptr<Node>> graph_; ///< List of the nodes in the maze graph
    };
} // namespace mazes

#endif
