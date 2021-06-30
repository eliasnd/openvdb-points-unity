#include <openvdb/openvdb.h>
#include <openvdb/points/PointConversion.h>
#include <openvdb/points/PointCount.h>
#include "utils.h"

using namespace std;
using namespace openvdb::points;

class OpenVDBPointsData
{
public:
    openvdb::points::PointDataGrid::Ptr gridPtr;

    OpenVDBPointsData(const char *filename, const char *gridName, LoggingCallback cb);
    OpenVDBPointsData(openvdb::points::PointDataGrid::Ptr ptr) { gridPtr = ptr; }
    OpenVDBPointsData(){};

    openvdb::Index64 pointCount();
    openvdb::Index64 populatePoints(openvdb::math::Mat4s worldToClipMatrix, bool frustumCulling, bool lod, Point *points, LoggingCallback cb);
};