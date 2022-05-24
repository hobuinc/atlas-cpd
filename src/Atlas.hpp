#pragma once

#include <pdal/PipelineManager.hpp>
#include <pdal/PointView.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include <Eigen/Dense>

#include "Grid.hpp"
#include "Types.hpp"

namespace AtlasProcessor
{

using StringList = pdal::StringList;

class Atlas
{
public:
    Atlas();

    void run(const StringList& s);

private:
    void addArgs();
    void load();
    void parse(const StringList& s);
    void throwError(const std::string& s);
    void write(const std::string& filename);
    
    pdal::ProgramArgs m_args;
    std::string m_beforeFilename;
    std::string m_afterFilename;
    int m_minpts;
    bool m_debug;
    std::unique_ptr<Grid> m_grid;

    StringList m_transformSpecs;
    Eigen::Matrix4d m_transform;
    pdal::PipelineManager m_beforeMgr;
    pdal::PipelineManager m_afterMgr;
    const double m_len = 100.0;
};

} // namespace
