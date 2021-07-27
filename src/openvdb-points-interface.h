#ifndef OPENVDB_POINTS_INTERFACE_H
#define OPENVDB_POINTS_INTERFACE_H

#include <openvdb/openvdb.h>
#include <openvdb/points/PointConversion.h>
#include "readply.h"
#include "openvdb-points-data.h"
using namespace std;
using namespace openvdb::points;

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
    void openvdbInitialize();
    void openvdbUninitialize();

    // Import methods
    bool convertPLYToVDB(const char *filename, const char *outfile, LoggingCallback cb);
    OpenVDBPointsData *readPointDataFromFile(const char *filename, const char *gridName, LoggingCallback cb);

    // OpenVDBPointsData operations
    int populatePoints(OpenVDBPointsData *data, Point *points);
    openvdb::Index64 getPointCountFromGrid(OpenVDBPointsData *reference);
    Index32_3 getTreeShape(OpenVDBPointsData *data);
    void destroyPointData(OpenVDBPointsData *reference);
    void populateTreeOffsets(OpenVDBPointsData *data, int *layer1Offsets, int *layer2Offsets, int *leafNodeOffsets);
    void populateTreeMask(OpenVDBPointsData *data, openvdb::math::Mat4s cam, bool frustumCulling, bool lod, bool occlusionCulling, int *internal1Mask, int *internal2Mask, int *layer3Mask, LoggingCallback cb);
}

void cloudToVDB(PLYReader::PointData<float, uint8_t> cloud, string filename);
openvdb::points::PointDataGrid::Ptr loadPointGrid(string filename, string gridName);

#endif