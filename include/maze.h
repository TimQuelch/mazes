#include <vector>

namespace mazes {
    constexpr unsigned DEFAULT_MAZE_SIZE = 21;

    class Maze {
    private:
        std::vector<std::vector<bool>> grid_;
        unsigned size_;

    public:
        Maze(unsigned size = DEFAULT_MAZE_SIZE);

        void print();
    };
} // namespace mazes
