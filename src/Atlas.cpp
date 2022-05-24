#include "Atlas.hpp"

#include <pdal/private/gdal/GDALUtils.hpp>
#include <pdal/private/gdal/Raster.hpp>
#include <pdal/util/FileUtils.hpp>

namespace AtlasProcessor
{

void fatal(const std::string& err)
{
    std::cerr << "atlas: " << err << "\n";
    exit(-1);
}

Atlas::Atlas() : m_transform(Eigen::Matrix4d::Identity())
{}

void Atlas::throwError(const std::string& s)
{
    throw std::runtime_error(s);
}

void Atlas::addArgs()
{
    m_args.add("before", "Filename of scene at time 't'",
        m_beforeFilename).setPositional();
    m_args.add("after", "Filename of scene at time 't + n'",
        m_afterFilename).setPositional();
    m_args.add("transform", "List of matrix entries - multiplied as"
        "written: A B C = A * B * C", m_transformSpecs).setOptionalPositional();
    m_args.add("minpts", "Minimum number of points in a cell to permit processing",
        m_minpts, 250);
    m_args.add("debug", "Dump transform and points", m_debug);
}

void Atlas::parse(const StringList& slist)
{
    try
    {
        m_args.parse(slist);
    }
    catch (const pdal::arg_error& err)
    {
        fatal(err.what());
    }

    for (std::string s : m_transformSpecs)
    {
        // Assume we have a filename;
        std::string fileInput = pdal::FileUtils::readFileIntoString(s);
        if (fileInput.size())
        {
            s = fileInput;
        }
        StringList l = pdal::Utils::split2(s,
            [](char c){return std::isspace((unsigned char)c); } );
        if (l.size() != 16)
            throwError("Each 'transform' option must have 16 numeric entries");

        Eigen::Matrix4d m;
        size_t pos = 0;
        for (size_t r = 0; r < 4; ++r)
        {
            for (size_t c = 0; c < 4; ++c)
            {
                std::string sval = l[pos++];
                char *end;
                pdal::Utils::trimTrailing(sval);
                double d = strtod(sval.data(), &end);
                if (d == HUGE_VAL || *end != '\0')
                    throwError("'transform' entry " + std::to_string(pos) +
                        " is not a valid numeric value.");
                m(r, c) = d;
            }
        }
        m_transform *= m;
    }
}

void Atlas::run(const StringList& s)
{
    addArgs();
    parse(s);
    try
    {
        load();
        m_grid->registration(m_minpts, m_debug);
        std::string filename = '/data/' + pdal::FileUtils::stem(m_beforeFilename) + ".out";

        write();
    }
    catch (const pdal::pdal_error& err)
    {
        fatal(err.what());
    }
}

void Atlas::load()
{
    using namespace pdal;

    Options transformOpts;
    std::stringstream ss;
    ss << m_transform;
    transformOpts.add("matrix", ss.str());

    StageCreationOptions bOps { m_beforeFilename };
    Stage& beforeReader = m_beforeMgr.makeReader(bOps);
    /**
    Stage& beforeFilter = m_beforeMgr.makeFilter("filters.transformation",
        beforeReader, transformOpts);
    **/
    m_beforeMgr.execute(ExecMode::Standard);

    StageCreationOptions aOps { m_afterFilename };
    Stage& afterReader = m_afterMgr.makeReader(aOps);
    /**
    Stage& afterFilter = m_afterMgr.makeFilter("filters.transformation",
        afterReader, transformOpts);
    **/
    m_afterMgr.execute(ExecMode::Standard);

    m_grid.reset(new Grid(m_len));
    PointViewPtr bp = *(m_beforeMgr.views().begin());
    m_grid->insert(bp, AP::Order::Before);

    PointViewPtr ap = *(m_afterMgr.views().begin());
    m_grid->insert(ap, AP::Order::After);

    m_grid->calcLimits();
}


void Atlas::write()
{
    using namespace pdal;

    const std::string filename("vector.out");

    std::array<double, 6> pixelToPos;
    pixelToPos[0] = (m_grid->xOrigin() * m_len);
    pixelToPos[1] = m_len;
    pixelToPos[2] = 0;
    pixelToPos[3] = (m_grid->yOrigin() * m_len);
    pixelToPos[4] = 0;
    pixelToPos[5] = m_len;

    gdal::registerDrivers();
    gdal::Raster raster(filename, "GTiff", "EPSG:32624", pixelToPos);
    gdal::GDALError err = raster.open(m_grid->xSize(), m_grid->ySize(),
        3, Dimension::Type::Float, -9999, pdal::StringList());

    if (err != gdal::GDALError::None)
        throwError(raster.errorMsg());

    raster.writeBand(GridIter(*m_grid, Dimension::Id::X), -9999.0, 1, "X");
    raster.writeBand(GridIter(*m_grid, Dimension::Id::Y), -9999.0, 2, "Y");
    raster.writeBand(GridIter(*m_grid, Dimension::Id::Z), -9999.0, 3, "Z");
}

} // namespace
