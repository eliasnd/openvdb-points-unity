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

extern "C"
{
    void openvdbInitialize();
    void openvdbUninitialize();
    bool convertPLYToVDB(string filename, string outfile);
}

void cloudToVDB(PLYReader::PointData<float, uint8_t> cloud, string filename);