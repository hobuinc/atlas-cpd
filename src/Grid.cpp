
#include <cpd/rigid.hpp>

#include "Grid.hpp"

namespace AtlasProcessor
{

void Grid::insert(pdal::PointViewPtr in, AP::Order order)
{
    using namespace pdal;
    using namespace pdal::Dimension;

    for (PointId id = 0; id < in->size(); ++id)
    {
        double x = in->getFieldAs<double>(Id::X, id);
        double y = in->getFieldAs<double>(Id::Y, id);
        int ix = int(std::floor(x / m_len));
        int iy = int(std::floor(y / m_len));

        GridIndex index(ix, iy);
        auto ci = m_cells.find(index);
        if (ci == m_cells.end())
        {
            GridCell cell(ix, iy, m_len, in);
            ci = m_cells.insert(std::make_pair(index, cell)).first;
        }
        GridCell& cell = ci->second;
        PointViewPtr& out = (order == Order::Before ?
            cell.m_before : cell.m_after);
        out->appendPoint(*in, id);
    }
}


void Grid::calcLimits()
{
    int xmin = (std::numeric_limits<int>::max)();
    int xmax = (std::numeric_limits<int>::lowest)();
    int ymin = (std::numeric_limits<int>::max)();
    int ymax = (std::numeric_limits<int>::lowest)();
    for (auto& cellPair : m_cells)
    {
        const GridIndex& idx = cellPair.first;
        int x = idx.x();
        int y = idx.y();
        xmin = (std::min)(xmin, x);
        xmax = (std::max)(xmax, x);
        ymin = (std::min)(ymin, y);
        ymax = (std::max)(ymax, y);
    }
    m_xOrigin = xmin;
    m_yOrigin = ymin;
    m_xSize = xmax - xmin + 1;
    m_ySize = ymax - ymin + 1;
}


void Grid::registration()
{
    for (auto ci = m_cells.begin(); ci != m_cells.end(); ++ci)
    {
        GridCell& cell = ci->second;
        const GridIndex& idx = ci->first;

        cell.registration();
    }
}


Eigen::Vector3d *Grid::getVector(int x, int y)
{
    auto ci = m_cells.find(GridIndex(x, y));
    if (ci == m_cells.end())
        return nullptr;
    return &(ci->second.m_vec);
}

//
// GridCell
//

void GridCell::registration()
{
    using namespace pdal::Dimension;

    if (!m_before || !m_after || m_before->size() < 250 || m_after->size() < 250)
    {
        std::cerr << "Aborting for " << m_x << "/" << m_y << ".\n";
        return;
    }
    std::cerr << "Computing for " << m_x << "/" << m_y << ".\n";

    // Convert points to Eigen Matrices.
    Eigen::MatrixX3d bm(m_before->size(), 3);
    for (size_t i = 0; i < m_before->size(); ++i)
    {
        double x, y, z;
        bm(i, 0) = m_before->getFieldAs<double>(Id::X, i);
        bm(i, 1) = m_before->getFieldAs<double>(Id::Y, i);
        bm(i, 2) = m_before->getFieldAs<double>(Id::Z, i);
    }
    Eigen::MatrixX3d am(m_after->size(), 3);
    for (size_t i = 0; i < m_after->size(); ++i)
    {
        am(i, 0) = m_after->getFieldAs<double>(Id::X, i);
        am(i, 1) = m_after->getFieldAs<double>(Id::Y, i);
        am(i, 2) = m_after->getFieldAs<double>(Id::Z, i);
    }

    // Find the average Z value to use for our velocity raster.
    auto lastcol = bm.col(bm.cols() - 1);
    double zMean = lastcol.mean();

    auto result = cpd::rigid(bm, am);

    Eigen::Matrix4d xform = result.matrix();
    Eigen::Matrix4d inv = xform.inverse();

    Eigen::Vector4d vec((m_x + .5) * m_len, (m_y + .5) * m_len, zMean, 1);

    // CPD creates a transformation from the _after_ (moving) set to the
    // _before_ (fixed) set. We want it the other way around, so we multiply
    // the inverse of the transform on the left by the point we want transformed.
    // We then subtract the original source vector to get actual movement.
    // The result is a 4x1 vector, so we trim it to 3x1.
    m_vec = ((inv * vec) - vec).head(3);
    std::cerr << "Vec = (" << m_vec(0) << ", " << m_vec(1) << ", " << m_vec(2) << ")\n";
}

//
// GridIter
//

GridIter::GridIter(Grid& grid, pdal::Dimension::Id dim) :
    m_grid(grid), m_pos(0), m_dimOffset(-1)
{
    using namespace pdal::Dimension;

    if (dim != Id::X && dim != Id::Y && dim != Id::Z &&
            dim != Id::Unknown)
        std::cerr << "Dimension must be X, Y, Z or Unknown.\n";
    m_dimOffset = static_cast<int>(dim) - 1;
}

int32_t GridIter::x() const
{
    return (m_pos % m_grid.xSize()) + m_grid.xOrigin();
}

int32_t GridIter::y() const
{
    return (m_pos / m_grid.xSize()) + m_grid.yOrigin();
}

GridIter& GridIter::operator++()
{
    m_pos++;
    return *this;
}


GridIter GridIter::operator++(int)
{
    GridIter it(*this);
    m_pos++;
    return it;
}

GridIter& GridIter::operator--()
{
    m_pos--;
    return *this;
}

GridIter GridIter::operator--(int)
{
    GridIter it(*this);
    m_pos--;
    return it;
}

GridIter GridIter::operator+(const difference_type& n) const
{
    GridIter it(*this);
    it.m_pos = m_pos + n;
    return it;
}


GridIter GridIter::operator-(const difference_type& n) const
{
    GridIter it(*this);
    it.m_pos = m_pos - n;
    return it;
}

const double& GridIter::operator*() const
{
    static double empty(-9999);

    Eigen::Vector3d *vec = m_grid.getVector(x(), y());
    if (!vec)
        return empty;
    return (*vec)(m_dimOffset);
}

double& GridIter::operator*()
{
    const double& d = const_cast<const GridIter *>(this)->operator *();
    return const_cast<double&>(d);
}

const double *GridIter::operator->() const
{
    static double empty(-9999);

    Eigen::Vector3d *vec = m_grid.getVector(x(), y());
    if (!vec)
        return &empty;
    return &(*vec)(m_dimOffset);
}

double *GridIter::operator->()
{
    const double& d = const_cast<const GridIter *>(this)->operator *();
    double& dd = const_cast<double &>(d);
    return &dd;
}

} // namespace
