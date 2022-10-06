#include "primitives.h"

#include <fstream>
#include <iostream>
#include <iterator>
#include <set>

using set_t = std::set<Point>;
template <typename ITER>
std::set<Point, bool (*)(const Point &, const Point &)> to_set(const std::pair<ITER, ITER> & range)
{
    auto cmp = [](const Point & a, const Point & b) {
        if (a.x() == b.x()) {
            return a.y() < b.y();
        }
        return a.x() < b.x();
    };
    std::set<Point, bool (*)(const Point &, const Point &)> res(cmp);
    std::copy(range.first, range.second, std::inserter(res, res.begin()));
    return res;
}
int main(int argc, char ** argv)
{
    if (argc != 4 && argc != 6) {
        std::cout << "Wrong amount of arguments. Provide filename and coordinates as arguments. See example below:\n";
        std::cout << "2d-tree test/etc/my_test.dat 1 1 3 5" << std::endl;
        return 0;
    }
    if (argc == 4) {
        Point point(std::atof(argv[2]), std::atof(argv[3]));
        // rb_tree running
        rbtree::PointSet rb_tree(argv[1]);
        std::cout << "rb_tree result: " << *rb_tree.nearest(point);
        // kd_tree running
        kdtree::PointSet kd_tree(argv[1]);
        std::cout << "kd_tree result: " << *kd_tree.nearest(point);
    }
    else {
        Point left_bottom(std::atof(argv[2]), std::atof(argv[3]));
        Point right_top(std::atof(argv[4]), std::atof(argv[5]));
        Rect rect(left_bottom, right_top);

        // rb_tree running
        rbtree::PointSet rb_tree(argv[1]);
        auto rb_set = to_set(rb_tree.range(rect));
        // kd_tree running
        kdtree::PointSet kd_tree(argv[1]);
        auto kd_set = to_set(kd_tree.range(rect));
        std::cout << "Comparing result from rb_tree and kd_tree:\n";
        auto it1 = rb_set.begin();
        std::size_t i = 1;
        for (auto it2 = kd_set.begin(); it2 != kd_set.end() && it1 != rb_set.end(); it2++, it1++) {
            if (*it1 != *it2) {
                std::cout << "Difference in results from rb_tree and kd_tree found in point" << i << ":\n"
                          << *it1 << *it2;
                return 0;
            }
            std::cout << i++ << ") " << *it1;
        }
    }

    kdtree::PointSet tree;
    tree.put({3, 4});
    tree.put({4, 0});
    auto ans = tree.nearest({0, 0});
    if (ans) {
        std::cout << ans.value() << "\n";
    }
}
