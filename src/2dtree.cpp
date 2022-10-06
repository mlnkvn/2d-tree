#include "primitives.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>

Point::Point(double x, double y)
    : x_coord(x)
    , y_coord(y)
{
}

double Point::distance(const Point & other) const
{
    return std::sqrt(std::pow(x_coord - other.x(), 2) +
                     std::pow(y_coord - other.y(), 2));
}

bool Point::operator<(const Point & p) const
{
    return x_coord < p.x() || y_coord < p.y();
}

bool Point::operator>(const Point & p) const
{
    return x_coord > p.x() || y_coord > p.y();
}

bool Point::operator<=(const Point & p) const
{
    return !(*this > p);
}

bool Point::operator>=(const Point & p) const
{
    return !(*this < p);
}

bool Point::operator==(const Point & p) const
{
    constexpr double eps = std::numeric_limits<double>::epsilon();
    return (std::abs(x_coord - p.x()) < eps) && (std::abs(y_coord - p.y()) < eps);
}

bool Point::operator!=(const Point & p) const
{
    return !(*this == p);
}

std::ostream & operator<<(std::ostream & strm, const Point & p)
{
    return strm << p.x() << " " << p.y() << std::endl;
}

Rect::Rect(const Point & left_bottom, const Point & right_top)
    : left_bottom({left_bottom.x(), left_bottom.y()})
    , right_top({right_top.x(), right_top.y()})
{
}

double Rect::distance(const Point & p) const
{
    if (p.x() >= xmin() && p.x() <= xmax()) {
        if (p.y() >= ymin() && p.y() <= ymax()) {
            return 0;
        }
        return std::min(std::abs(p.y() - ymin()), std::abs(p.y() - ymax()));
    }
    return std::min(std::abs(p.x() - xmin()), std::abs(p.x() - xmax()));
}

bool Rect::contains(const Point & p) const
{
    constexpr double eps = std::numeric_limits<double>::epsilon();
    return std::abs(distance(p)) < eps;
}

bool Rect::intersects(const Rect & rect) const
{
    if ((rect.xmax() - xmin()) * (rect.xmin() - xmax()) <= 0) {
        if ((rect.ymax() - ymin()) * (rect.ymin() - ymax()) <= 0) {
            return true;
        }
    }
    return false;
}

namespace rbtree {

PointSet::PointSet(const std::string & filename)
{
    std::ifstream fs(filename);
    if (!fs.good()) {
        return;
    }
    double x, y;
    while (fs) {
        fs >> x >> y;
        if (fs.fail()) {
            break;
        }
        m_set.insert(Point(x, y));
    }
    fs.close();
}

bool PointSet::empty() const
{
    return m_set.empty();
}

std::size_t PointSet::size() const
{
    return m_set.size();
}

void PointSet::put(const Point & p)
{
    m_set.insert(p);
}

bool PointSet::contains(const Point & p) const
{
    return m_set.find(p) != m_set.end();
}

PointSet::PointSet(const std::set<Point> & set)
    : m_set(set)
{
}

std::pair<PointSet::iterator, PointSet::iterator> PointSet::range(const Rect & rect) const
{
    auto in_rect = std::make_shared<std::vector<Point>>();
    for (auto it = begin(); it != end(); it++) {
        if (rect.contains(*it)) {
            in_rect->push_back(*it);
        }
    }
    return {iterator(in_rect), iterator(in_rect->end())};
}

PointSet::iterator PointSet::begin() const
{
    return rbtree::PointSet::iterator(*this);
}

PointSet::iterator PointSet::end() const
{
    return PointSet::iterator(m_set.end());
}

std::optional<Point> PointSet::nearest(const Point & point) const
{
    return *std::min_element(begin(), end(), [&point](const Point & a, const Point & b) { return a.distance(point) < b.distance(point); });
}

std::pair<PointSet::iterator, PointSet::iterator> PointSet::nearest(const Point & point, std::size_t k) const
{
    if (k >= m_set.size()) {
        return {begin(), end()};
    }
    if (k == 0) {
        return {begin(), begin()};
    }
    auto neighbours = std::make_shared<std::vector<Point>>();
    neighbours->reserve(k);
    for (auto it = begin(); it != end(); it++) {
        if (neighbours->size() < k) {
            neighbours->push_back(*it);
        }
        else {
            auto max_distanced = std::max_element(neighbours->begin(), neighbours->end(), [&point](const Point & lhs, const Point & rhs) { return lhs.distance(point) <= rhs.distance(point); });
            if (it->distance(point) < max_distanced->distance(point)) {
                *max_distanced = *it;
            }
        }
    }
    return {{neighbours}, {neighbours->end()}};
}

std::ostream & operator<<(std::ostream & strm, const rbtree::PointSet & points)
{
    for (auto it = points.begin(); it != points.end(); it++) {
        strm << *it;
    }
    return strm;
}

} // namespace rbtree

namespace kdtree {

PointSet::PointSet(const std::string & filename)
{
    std::ifstream fs(filename);
    if (!fs.is_open()) {
        return;
    }
    std::vector<Point> points;
    double x, y;
    while (fs) {
        fs >> x >> y;
        if (fs.fail()) {
            break;
        }
        points.emplace_back(Point(x, y));
    }
    fs.close();
    buildTree(this, points, 0, points.size(), 0);
}

void PointSet::buildTree(PointSet * tree, std::vector<Point> & points, std::size_t start, std::size_t end, std::size_t depth) const
{
    if (end - start < 1) {
        return;
    }
    std::size_t middle = (end + start) / 2;
    auto m = points.begin() + middle;
    std::nth_element(points.begin() + start, m, points.begin() + end, [&depth](const Point & lhs, const Point & rhs) {
        return (depth % 2 == 0) ? (lhs.x() < rhs.x()) : (lhs.y() < rhs.y());
    });
    tree->m_root = tree->insert(*m, tree->m_root, 0);
    buildTree(tree, points, start, middle, depth + 1);
    buildTree(tree, points, middle + 1, end, depth + 1);
}

bool PointSet::empty() const
{
    return m_root == nullptr;
}

std::size_t PointSet::size() const
{
    return m_size;
}

const PointSet::NodePtr & PointSet::insert(const Point & point, NodePtr & node, std::size_t depth)
{
    max_depth = std::max(max_depth, depth);
    if (node == nullptr) {
        node = std::make_shared<Node>(point);
        node->depth = depth;
        ++m_size;
        return node;
    }
    if (*node->m_point == point) {
        return node;
    }
    bool toLeft = (node->depth % 2 == 0) ? (point.x() <= node->m_point->x()) : (point.y() <= node->m_point->y());
    if (toLeft) {
        node->left = insert(point, node->left, depth + 1);
        node->left->parent = node;
    }
    else {
        node->right = insert(point, node->right, depth + 1);
        node->right->parent = node;
    }
    return node;
}

void PointSet::reBuild()
{
    if (max_depth > 2 * std::log(m_size)) {
        std::vector<Point> points;
        points.reserve(m_size);
        for (auto it = begin(); it != end(); it++) {
            points.emplace_back(it->x(), it->y());
        }
        m_size = 0;
        max_depth = 0;
        m_root = nullptr;
        buildTree(this, points, 0, points.size(), 0);
    }
}

void PointSet::put(const Point & p)
{
    m_root = insert(p, m_root, 0);
    reBuild();
}

const PointSet::NodePtr & PointSet::find(const Point & p, const NodePtr & node) const
{
    if (node == nullptr) {
        return node;
    }
    if (*node->m_point == p) {
        return node;
    }
    bool toLeft = (node->depth % 2 == 0) ? (p.x() <= node->m_point->x()) : (p.y() <= node->m_point->y());
    if (toLeft) {
        return find(p, node->left);
    }
    return find(p, node->right);
}

bool PointSet::contains(const Point & p) const
{
    return find(p, m_root) != nullptr;
}

void PointSet::findPointsInRectangle(const NodePtr & node, std::vector<Point> & points, const Rect & rect) const
{
    if (node == nullptr) {
        return;
    }
    if (rect.contains(*node->m_point)) {
        points.push_back(*node->m_point);
    }
    bool toLeft = (node->depth % 2 == 0) ? (node->m_point->x() >= rect.xmin()) : (node->m_point->y() >= rect.ymin());
    bool toRight = (node->depth % 2 == 0) ? (node->m_point->x() <= rect.xmax()) : (node->m_point->y() <= rect.ymax());
    if (toLeft) {
        findPointsInRectangle(node->left, points, rect);
    }
    if (toRight) {
        findPointsInRectangle(node->right, points, rect);
    }
}

std::pair<PointSet::iterator, PointSet::iterator> PointSet::range(const Rect & rect) const
{
    auto in_rect = std::make_shared<std::vector<Point>>();
    findPointsInRectangle(m_root, *in_rect, rect);
    return {{in_rect}, {in_rect->end()}};
}

PointSet::iterator PointSet::begin() const
{
    return {left(m_root), *this};
}

PointSet::iterator PointSet::end() const
{
    return {*this};
}

void PointSet::findNeighbour(const NodePtr & node, const Point & point, Point & closest_found) const
{
    if (node == nullptr) {
        return;
    }
    double dist = node->m_point->distance(point);
    if (dist < point.distance(closest_found)) {
        closest_found = *node->m_point;
    }
    if (dist == 0) {
        return;
    }
    double delta;
    if (node->depth % 2 == 0) {
        delta = node->m_point->x() - point.x();
    }
    else {
        delta = node->m_point->y() - point.y();
    }
    findNeighbour((delta > 0) ? node->left : node->right, point, closest_found);
    if (std::abs(delta) >= point.distance(closest_found)) {
        return;
    }
    findNeighbour((delta > 0) ? node->right : node->left, point, closest_found);
}

std::optional<Point> PointSet::nearest(const Point & point) const
{
    Point closest_point(*m_root->m_point);
    findNeighbour(m_root, point, closest_point);
    return std::optional<Point>(closest_point);
}

std::pair<PointSet::iterator, PointSet::iterator> PointSet::nearest(const Point & p, std::size_t k) const
{
    if (k >= m_size) {
        return {begin(), end()};
    }
    if (k == 0) {
        return {begin(), begin()};
    }
    auto neighbours = std::make_shared<std::vector<Point>>();
    neighbours->reserve(k);
    for (auto it = begin(); it != end(); it++) {
        if (neighbours->size() < k) {
            neighbours->push_back(*it);
        }
        else {
            auto max_distanced = std::max_element(neighbours->begin(), neighbours->end(), [&p](const Point & lhs, const Point & rhs) { return lhs.distance(p) <= rhs.distance(p); });
            if (it->distance(p) < max_distanced->distance(p)) {
                *max_distanced = *it;
            }
        }
    }
    return {{neighbours}, {neighbours->end()}};
}

std::ostream & operator<<(std::ostream & strm, const PointSet & points)
{
    for (auto it = points.begin(); it != points.end(); it++) {
        strm << *it;
    }
    return strm;
}

PointSet::NodePtr PointSet::next(NodePtr & point) const
{
    return next(m_root, point);
}

PointSet::NodePtr PointSet::next(const NodePtr & root, NodePtr & node)
{
    if (node == nullptr) {
        return nullptr;
    }
    if (node->right != nullptr) {
        return left(node->right);
    }
    bool isRight;
    NodePtr parent = node->parent.lock();
    while (parent != nullptr && parent->right == node) {
        node = parent;
        parent = parent->parent.lock();
        isRight = true;
    }
    if (node == root && isRight) {
        return nullptr;
    }
    parent = node->parent.lock();
    if (parent != nullptr) {
        return parent;
    }
    return node;
}

PointSet::NodePtr PointSet::left(const NodePtr & node)
{
    if (node != nullptr && node->left != nullptr) {
        return left(node->left);
    }
    return node;
}

const PointSet::NodePtr & PointSet::copyTree(const NodePtr & from, NodePtr & to)
{
    if (from == nullptr) {
        return from;
    }
    to = std::make_shared<Node>(*from->m_point);
    to->depth = from->depth;
    if (from->left == nullptr && from->right == nullptr) {
        return to;
    }
    to->left = copyTree(from->left, to->left);
    if (to->left != nullptr) {
        to->left->parent = to;
    }
    to->right = copyTree(from->right, to->right);
    if (to->right != nullptr) {
        to->right->parent = to;
    }
    return to;
}

PointSet::PointSet(const PointSet & set)
    : max_depth(set.max_depth)
    , m_root(nullptr)
    , m_size(set.m_size)
{
    copyTree(set.m_root, m_root);
}
} // namespace kdtree
