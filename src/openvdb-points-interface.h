#ifndef OPENVDB_POINTS_INTERFACE_H
#define OPENVDB_POINTS_INTERFACE_H

#include <openvdb/openvdb.h>
#include <openvdb/points/PointConversion.h>
#include "readply.h"
#include "openvdb-points-data.h"
using namespace std;
using namespace openvdb::points;

static bool init;

// static void UNITY_INTERFACE_API OnGraphicsDeviceEvent(UnityGfxDeviceEventType eventType);

struct RenderingData
{
    OpenVDBPointsData *dataPtr;
    Point *pointsPtr;
    openvdb::math::Mat4s worldToClipMatrix;
    bool frutumCulling;
    bool lod;
    LoggingCallback cb;
};

extern "C"
{
    // UnityRenderingEventAndData UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API GetRenderEventFunc();

    void openvdbInitialize();
    void openvdbUninitialize();
    bool convertPLYToVDB(const char *filename, const char *outfile, LoggingCallback cb);
    OpenVDBPointsData *readPointDataFromFile(const char *filename, const char *gridName, LoggingCallback cb);
    int populatePoints(OpenVDBPointsData *data, Point *points);
    openvdb::Index64 getPointCountFromGrid(OpenVDBPointsData *reference);
    void destroyPointData(OpenVDBPointsData *reference);
}

void cloudToVDB(PLYReader::PointData<float, uint8_t> cloud, string filename);
openvdb::points::PointDataGrid::Ptr loadPointGrid(string filename, string gridName);

#endif