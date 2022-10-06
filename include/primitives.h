#pragma once

#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>

class Point
{
    double x_coord;
    double y_coord;

public:
    Point(double x, double y);
    double x() const
    {
        return x_coord;
    }
    double y() const
    {
        return y_coord;
    }
    double distance(const Point &) const;

    bool operator<(const Point &) const;
    bool operator>(const Point &) const;
    bool operator<=(const Point &) const;
    bool operator>=(const Point &) const;
    bool operator==(const Point &) const;
    bool operator!=(const Point &) const;

    friend std::ostream & operator<<(std::ostream &, const Point &);
};

class Rect
{
    Point left_bottom;
    Point right_top;

public:
    Rect(const Point & left_bottom, const Point & right_top);
    double xmin() const
    {
        return left_bottom.x();
    }
    double ymin() const
    {
        return left_bottom.y();
    }

    double xmax() const
    {
        return right_top.x();
    }

    double ymax() const
    {
        return right_top.y();
    }
    double distance(const Point & p) const;

    bool contains(const Point & p) const;
    bool intersects(const Rect &) const;
};

using VectorIterator = std::vector<Point>::iterator;
using SetIterator = std::set<Point>::iterator;
using PointVectorPtr = std::shared_ptr<std::vector<Point>>;

namespace rbtree {

class PointSet
{
    std::set<Point> m_set;

public:
    class iterator
    {
    public:
        using difference_type = std::ptrdiff_t;
        using value_type = Point;
        using pointer = const value_type *;
        using reference = const value_type &;
        using iterator_category = std::forward_iterator_tag;
        iterator() = default;
        friend bool operator==(const iterator & lhs, const iterator & rhs)
        {
            return lhs.m_it == rhs.m_it;
        }
        friend bool operator!=(const iterator & lhs, const iterator & rhs)
        {
            return lhs.m_it != rhs.m_it;
        }
        reference operator*() const
        {
            if (std::holds_alternative<VectorIterator>(m_it)) {
                return *std::get<VectorIterator>(m_it);
            }
            return *std::get<SetIterator>(m_it);
        }
        pointer operator->() const
        {
            if (std::holds_alternative<VectorIterator>(m_it)) {
                return &*std::get<VectorIterator>(m_it);
            }
            return &*std::get<SetIterator>(m_it);
        }
        iterator & operator++()
        {
            std::visit([&](auto & it) { ++it; }, m_it);
            return *this;
        }
        iterator operator++(int)
        {
            auto tmp = *this;
            operator++();
            return tmp;
        }

    private:
        friend class PointSet;
        iterator(const PointVectorPtr & points)
            : m_it(points->begin())
            , m_points(points)
        {
        }
        iterator(const SetIterator & it)
            : m_it(it)
        {
        }
        iterator(const VectorIterator & it)
            : m_it(it)
        {
        }
        iterator(const PointSet & points)
            : m_it(points.m_set.begin())
            , m_points(&points)
        {
        }
        std::variant<VectorIterator, SetIterator> m_it;
        std::variant<PointVectorPtr, const PointSet *> m_points;
    };

    PointSet(const std::string & filename = {});
    PointSet(const std::set<Point> & set);
    bool empty() const;
    std::size_t size() const;
    void put(const Point &);
    bool contains(const Point &) const;

    // second iterator points to an element out of range
    std::pair<iterator, iterator> range(const Rect &) const;
    iterator begin() const;
    iterator end() const;

    std::optional<Point> nearest(const Point &) const;
    // second iterator points to an element out of range
    std::pair<iterator, iterator> nearest(const Point & p, std::size_t k) const;

    friend std::ostream & operator<<(std::ostream &, const PointSet &);
};

} // namespace rbtree

namespace kdtree {

class PointSet
{
    struct Node
    {

        std::shared_ptr<Node> left = nullptr;
        std::shared_ptr<Node> right = nullptr;
        std::weak_ptr<Node> parent;
        std::unique_ptr<Point> m_point = nullptr;
        std::size_t depth = 0;
        Node(const Point & point)
            : m_point(std::make_unique<Point>(point))
        {
            parent.reset();
        }
    };
    using NodePtr = std::shared_ptr<Node>;

    NodePtr next(NodePtr & point) const;
    static NodePtr next(const NodePtr & root, NodePtr & cur_root);

public:
    class iterator
    {
    public:
        using difference_type = std::ptrdiff_t;
        using value_type = Point;
        using pointer = const value_type *;
        using reference = const value_type &;
        using iterator_category = std::forward_iterator_tag;

        iterator() = default;
        friend bool operator==(const iterator & lhs, const iterator & rhs)
        {
            return lhs.m_current == rhs.m_current;
        }
        friend bool operator!=(const iterator & lhs, const iterator & rhs)
        {
            return lhs.m_current != rhs.m_current;
        }
        reference operator*() const
        {
            if (std::holds_alternative<VectorIterator>(m_current)) {
                return *std::get<VectorIterator>(m_current);
            }
            return *std::get<NodePtr>(m_current)->m_point;
        }
        pointer operator->() const
        {
            if (std::holds_alternative<VectorIterator>(m_current)) {
                return &*std::get<VectorIterator>(m_current);
            }
            return &*std::get<NodePtr>(m_current)->m_point;
        }
        iterator & operator++()
        {
            if (std::holds_alternative<VectorIterator>(m_current)) {
                ++std::get<VectorIterator>(m_current);
            }
            else {
                m_current = std::get<const PointSet *>(m_points)->next(std::get<NodePtr>(m_current));
            }
            return *this;
        }
        iterator operator++(int)
        {
            auto tmp = *this;
            operator++();
            return tmp;
        }

    private:
        friend class PointSet;
        iterator(const PointSet & point_set)
            : m_current(std::shared_ptr<Node>(nullptr))
            , m_points(&point_set)
        {
        }
        iterator(const NodePtr & node, const PointSet & point_set)
            : m_current(node)
            , m_points(&point_set)
        {
        }
        iterator(const PointVectorPtr & point_set)
            : m_current(point_set->begin())
            , m_points(point_set)
        {
        }
        iterator(const VectorIterator & it)
            : m_current(it)
        {
        }
        std::variant<VectorIterator, NodePtr> m_current;
        std::variant<PointVectorPtr, const PointSet *> m_points = nullptr;
    };

    PointSet(const std::string & filename = {});
    PointSet(const PointSet & set);
    bool empty() const;
    std::size_t size() const;
    void put(const Point &);
    bool contains(const Point &) const;

    std::pair<iterator, iterator> range(const Rect &) const;
    iterator begin() const;
    iterator end() const;

    std::optional<Point> nearest(const Point &) const;
    std::pair<iterator, iterator> nearest(const Point & p, std::size_t k) const;

    friend std::ostream & operator<<(std::ostream &, const PointSet &);

private:
    std::size_t max_depth = 0;
    NodePtr m_root = nullptr;
    std::size_t m_size = 0;

    void reBuild();
    const NodePtr & insert(const Point & p, NodePtr & current, std::size_t depth);
    static NodePtr left(const NodePtr & current);
    const NodePtr & find(const Point & p, const NodePtr & current) const;
    void buildTree(PointSet * tree, std::vector<Point> & points, std::size_t start, std::size_t end, std::size_t depth) const;
    void findNeighbour(const NodePtr & root, const Point & point, Point & closest_found) const;
    void findPointsInRectangle(const NodePtr & root, std::vector<Point> & points, const Rect & rect) const;
    const NodePtr & copyTree(const NodePtr & from, NodePtr & to);
};

} // namespace kdtree
