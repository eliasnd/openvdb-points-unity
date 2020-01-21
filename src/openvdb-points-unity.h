#include <openvdb/openvdb.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <exception>
#include <openvdb/openvdb.h>
#include <openvdb/points/PointConversion.h>
#include <openvdb/points/PointCount.h>
#include "readply.h"
using namespace std;

typedef void (*LoggingCallback)(const char *message);

class SharedPointDataGridReference
{
public:
    openvdb::points::PointDataGrid::Ptr gridPtr;
    SharedPointDataGridReference(openvdb::points::PointDataGrid::Ptr ptr) { gridPtr = ptr; }
    SharedPointDataGridReference(){};
};

extern "C"
{
    void openvdbInitialize();
    void openvdbUninitialize();
    bool convertPLYToVDB(const char *filename, const char *outfile, LoggingCallback cb);
    SharedPointDataGridReference *readPointGridFromFile(const char *filename, const char *gridName, LoggingCallback cb);
    openvdb::Index64 getPointCountFromGrid(SharedPointDataGridReference *reference);
}

void cloudToVDB(PLYReader::PointData<float, uint8_t> cloud, string filename);
openvdb::points::PointDataGrid::Ptr loadPointGrid(string filename, string gridName);