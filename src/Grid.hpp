#pragma once

#include <unordered_map>

#include <Eigen/Dense>
#include <pdal/PointView.hpp>

#include "Types.hpp"

namespace AtlasProcessor
{

struct GridIndex
{
public:
    GridIndex(int32_t x, int32_t y) : m_key(key(x, y))
    { assert(x == this->x()); assert(y == this->y()); }

    int32_t x() const
    { return (uint32_t)m_key; }

    int32_t y() const
    { return (uint32_t)(m_key >> 32); }

    uint64_t key() const
    { return m_key; }

    bool operator==(const GridIndex& other) const
    { return m_key == other.m_key; }

private:
    uint64_t key(int32_t x, int32_t y)
    {
        uint32_t ux = (uint32_t)x;
        uint32_t uy = (uint32_t)y;
        return (ux | ((uint64_t)uy << 32));
    }

    uint64_t m_key;
};

}

namespace std
{
    template<>
    struct hash<AtlasProcessor::GridIndex>
    {
        using argument_type = AtlasProcessor::GridIndex;
        using result_type = std::size_t;

        result_type operator()(const AtlasProcessor::GridIndex& val) const
        { return (result_type)val.key(); }
    };
}


namespace AtlasProcessor
{

class Grid;

struct GridCell
{
    int m_x;
    int m_y;

    int m_len;
    pdal::PointViewPtr m_before;
    pdal::PointViewPtr m_after;
    Eigen::Vector3d m_vec;

    GridCell(pdal::PointTableRef table, int x, int y, int len, pdal::PointViewPtr inView) :
        m_x(x), m_y(y), m_len(len)
    {
        using namespace pdal;

        m_before = std::make_shared<PointView>(table);
        m_after = std::make_shared<PointView>(table);
    }
    void registration(int minpts, bool debug);
};

class Grid
{
public:
    Grid(int len) : m_len(len),
        m_xSize(std::numeric_limits<int>::lowest()),
        m_ySize(std::numeric_limits<int>::lowest()),
        m_xOrigin(std::numeric_limits<int>::lowest()),
        m_yOrigin(std::numeric_limits<int>::lowest())
    {
        using namespace pdal;

        m_table.layout()->registerDims( {Dimension::Id::X, Dimension::Id::Y, Dimension::Id::Z} );
        m_table.finalize();
    }

    void insert(pdal::PointViewPtr in, AP::Order order);
    Eigen::Vector3d *getVector(int x, int y);
    void registration(int minpts, bool debug);
    void calcLimits();

    size_t xSize()
        { return m_xSize; }
    size_t ySize()
        { return m_ySize; }
    size_t xOrigin()
        { return m_xOrigin; }
    size_t yOrigin()
        { return m_yOrigin; }

private:
    int m_len;
    int m_xSize;
    int m_ySize;
    int m_xOrigin;
    int m_yOrigin;
    std::unordered_map<GridIndex, GridCell> m_cells;
    pdal::PointTable m_table;
};

class GridIter
{
public:
    GridIter(Grid& grid, pdal::Dimension::Id dim);
    using iterator_category = std::random_access_iterator_tag;
    using difference_type = int32_t;
    using value_type = double;
    using pointer = double*;
    using reference = double&;

private:
    Grid& m_grid;
    uint64_t m_pos;
    int m_dimOffset; // Offset to vector iterator.

    int32_t x() const;
    int32_t y() const;

public:
    GridIter& operator++();
    GridIter operator++(int);
    GridIter operator+(const difference_type& n) const;
    GridIter& operator--();
    GridIter operator--(int);
    GridIter operator-(const difference_type& n) const;
    bool operator==(const GridIter& other)
        { return m_pos == other.m_pos; }
    bool operator!=(const GridIter& other)
        { return !(operator==(other)); }
    const double& operator*() const;
    double& operator*();
    const double *operator->() const;
    double *operator->();
};

} // namespace
