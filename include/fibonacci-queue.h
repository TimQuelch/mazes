#ifndef MAZES_FIBONACCI_QUEUE_H
#define MAZES_FIBONACCI_QUEUE_H

#include <list>
#include <map>
#include <memory>

namespace mazes {
    template <typename T, typename P>
    class FibonacciQueue;

    namespace detail {
        template <typename T, typename P>
        using FibNode = typename FibonacciQueue<T, P>::Node;

        template <typename T, typename P>
        using FibNodeList = std::list<std::unique_ptr<FibNode<T, P>>>;

        template <typename T, typename P>
        void reduceRoots(FibNodeList<T, P>& roots) {
            std::map<int, typename FibNodeList<T, P>::iterator> trees;
            for (auto it = roots.begin(); it != roots.end(); it++) {
                if (trees[(*it)->degree]) {
                    auto other = trees[(*it)->degree];
                    trees[(*it)->degree] = nullptr;
                    if (*other < *it) {
                        std::swap(*it, *other);
                    }
                    (*it)->children.splice((*it)->children.end(), roots, other);
                    (*it)->degree++;
                    it--;
                } else {
                    trees[(*it)->degree] = it;
                }
            }
        }
    } // namespace detail

    template <typename T, typename P>
    class FibonacciQueue {
    public:
        T const& top() const { return (*minVal_)->value; }
        bool empty() const { return false; } // TODO
        size_t size() const { return 0; }    // TODO

        void push(T const& value, int priority) {
            roots_.push_back(std::make_unique<Node>(value, priority));
            auto newNode = --(roots_.end());
            if (*newNode < *minVal_) {
                minVal_ = newNode;
            }
        }

        void pop() {
            roots_.splice(roots_.begin(), (*minVal_)->children);
            roots_.erase(minVal_);
            detail::reduceRoots(roots_);
            minVal_ = roots_.begin();
            for (auto it = roots_.begin(); it != roots_.end(); it++) {
                if (*it < *minVal_) {
                    minVal_ = it;
                }
            }
        }

    private:
        struct Node {
            T value;
            P priority;
            int degree;
            std::list<std::unique_ptr<Node>> children;

            Node(T const& value, P priority)
                : value{value}
                , priority{priority}
                , degree{0}
                , children{} {}

            bool operator<(Node const& other) { return priority < other.priority; }
        };

        std::list<std::unique_ptr<Node>> roots_;
        typename std::list<std::unique_ptr<Node>>::iterator minVal_;
    };
} // namespace mazes

#endif
