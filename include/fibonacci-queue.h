#ifndef MAZES_FIBONACCI_QUEUE_H
#define MAZES_FIBONACCI_QUEUE_H

#include <cstddef>
#include <list>
#include <map>
#include <memory>

namespace mazes {
    template <typename T, typename Compare>
    class FibonacciQueue {
    public:
        T const& top() const { return (*minVal_)->value; }
        bool empty() const { return false; } // TODO
        size_t size() const { return 0; }    // TODO

        void push(T const& value) { roots_.push_back(std::make_unique<Node>(value)); }

        void pop() {
            roots_.splice(roots_.begin(), (*minVal_)->children);
            roots_.erase(minVal_);
            reduceRoots(roots_, cmp_);
            minVal_ = roots_.begin();
            for (auto it = roots_.begin(); it != roots_.end(); it++) {
                if (cmp_(*it, *minVal_)) {
                    minVal_ = it;
                }
            }
        }

        FibonacciQueue(Compare cmp)
            : cmp_{cmp}
            , roots_{}
            , minVal_{} {}

    private:
        struct Node {
            T value;
            int degree;
            std::list<std::unique_ptr<Node>> children;

            Node(T const& value)
                : value{value}
                , degree{0}
                , children{} {}
        };

        Compare cmp_;
        std::list<std::unique_ptr<Node>> roots_;
        typename std::list<std::unique_ptr<Node>>::iterator minVal_;
    };

    template <typename Node, typename Compare>
    void reduceRoots(std::list<std::unique_ptr<Node>>& roots, Compare cmp) {
        std::map<int, typename std::list<std::unique_ptr<Node>>::iterator> trees;
        for (auto it = roots.begin(); it != roots.end(); it++) {
            if (trees[(*it)->degree]) {
                auto other = trees[(*it)->degree];
                trees[(*it)->degree] = nullptr;
                if (!cmp(*it, *other)) {
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
} // namespace mazes

#endif
