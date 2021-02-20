#include <openvdb/openvdb.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <exception>
#include <openvdb/openvdb.h>
#include <openvdb/tools/VolumeToMesh.h>
#include <openvdb/points/PointConversion.h>
#include <openvdb/points/PointCount.h>
#include <openvdb/tools/ParticlesToLevelSet.h>
#include <openvdb/tools/GridTransformer.h>
#include <openvdb/Grid.h>
#include <openvdb/tools/Clip.h>
#include <openvdb/math/BBox.h>
#include "particle-list-wrapper.h"
#include "readply.h"
using namespace std;

typedef void (*LoggingCallback)(const char *message);

class SharedPointDataGridReference
{
public:
    openvdb::points::PointDataGrid::Ptr gridPtr;
    vector<openvdb::Vec3s> meshPoints;
    vector<openvdb::Vec3I> meshTriangles;
    SharedPointDataGridReference(openvdb::points::PointDataGrid::Ptr ptr) { gridPtr = ptr; }
    SharedPointDataGridReference(){};
};

enum SampleQuality
{
    High = 1,
    Medium = 2,
    Low = 3
};

struct Vec3d
{
    double x;
    double y;
    double z;
};

struct Frustum  // Probably will need to change this for marshaling
{
    Vec3d pos;
    Vec3d dir;
    Vec3d up;
    double nearWorldX;  // X width of near plane in world space
    double aspect;
    double nPlane;
    double fPlane;
};

extern "C"
{
    void openvdbInitialize();
    void openvdbUninitialize();
    bool convertPLYToVDB(const char *filename, const char *outfile, LoggingCallback cb);
    SharedPointDataGridReference *readPointGridFromFile(const char *filename, const char *gridName, LoggingCallback cb);
    openvdb::Index64 getPointCountFromGrid(SharedPointDataGridReference *reference);
    void destroySharedPointDataGridReference(SharedPointDataGridReference *reference);
    Vec3d* generatePointArrayFromPointGrid(SharedPointDataGridReference *reference, LoggingCallback cb);
    SharedPointDataGridReference *arraysToPointGrid(Vec3d *positionArr, Vec3d *colorArr, uint count);
    openvdb::math::NonlinearFrustumMap *mapFromFrustum(Frustum frustum, openvdb::math::Vec3d voxelSize);
    SharedPointDataGridReference *occlusionMask(SharedPointDataGridReference *reference, Frustum frustum);
}

void cloudToVDB(PLYReader::PointData<float, uint8_t> cloud, string filename);
openvdb::points::PointDataGrid::Ptr loadPointGrid(string filename, string gridName);

template<typename GridType>
typename GridType::Ptr downsampleGrid(typename GridType::Ptr inGrid, SampleQuality quality);