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

struct Pos
{
    float x;
    float y;
    float z;
};

struct Color
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t a;
};

extern "C"
{
    void openvdbInitialize();
    void openvdbUninitialize();
    bool convertPLYToVDB(const char *filename, const char *outfile, LoggingCallback cb);
    SharedPointDataGridReference *readPointGridFromFile(const char *filename, const char *gridName, LoggingCallback cb);
    openvdb::Index64 getPointCountFromGrid(SharedPointDataGridReference *reference);
    void computeMeshFromPointGrid(SharedPointDataGridReference *reference, size_t &pointCount, size_t &triCount, LoggingCallback cb);
    void destroySharedPointDataGridReference(SharedPointDataGridReference *reference);
    void populatePointArraysFromPointGrid(Pos *posArr, Color *colArr, SharedPointDataGridReference *reference, LoggingCallback cb); 
}

void cloudToVDB(PLYReader::PointData<float, uint8_t> cloud, string filename);
openvdb::points::PointDataGrid::Ptr loadPointGrid(string filename, string gridName);

template<typename GridType>
typename GridType::Ptr downsampleGrid(typename GridType::Ptr inGrid, SampleQuality quality);