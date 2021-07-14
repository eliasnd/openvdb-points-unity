#pragma once

#include <openvdb/openvdb.h>
#include <openvdb/points/PointConversion.h>
#include "utils.h"

using namespace std;
using namespace openvdb::points;

class OpenVDBPointsInstance
{
    Point ***pointTree;
    bool **cullingMask;
};