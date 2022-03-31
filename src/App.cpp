#include "Atlas.hpp"

int main(int argc, const char *argv[])
{
    pdal::StringList slist(argv + 1, argv + argc);

    AtlasProcessor::Atlas atlas;
    atlas.run(slist);
}
