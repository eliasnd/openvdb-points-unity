#pragma once

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

    OpenVDBPointsData(const char *filename, const char *gridName, LoggingCallback cb)
    {
        try
        {
            string filePath(filename);
            string message = "Reading PointDataGrid from " + filePath;
            cb(message.c_str());

            string grid(gridName);

            if (grid.compare("") == 0)
            {
                openvdb::io::File file(filename);
                file.open();

                grid = file.beginName().gridName();

                string message2 = "Loading Grid " + grid;
                cb(message2.c_str());
            }

            openvdb::io::File fileHandle(filename);
            fileHandle.open();
            gridPtr = openvdb::gridPtrCast<PointDataGrid>(fileHandle.readGrid(gridName));
            fileHandle.close();
        }
        catch (exception &e)
        {
            cb(e.what());
        }
    };

    OpenVDBPointsData(openvdb::points::PointDataGrid::Ptr ptr) { gridPtr = ptr; }
    OpenVDBPointsData(){};

    openvdb::Index64 pointCount()
    {
        openvdb::Index64 count = openvdb::points::pointCount(gridPtr->tree());
        return count;
    };
    
    // NEW IDEA:
    // Millions of points too many to populate every frame
    // Just compute active voxels by checking each for frustum culling
    // If any voxel dirty, update buffer

    openvdb::Index64 populatePoints(openvdb::math::Mat4s worldToClipMatrix, bool frustumCulling, bool lod, Point *points, LoggingCallback cb)
    {
        openvdb::Real voxelSize = gridPtr->voxelSize().x();

        auto bboxCornersWorldSpace = [this](openvdb::math::CoordBBox bbox, openvdb::Vec3f *corners)
        {
            openvdb::math::Coord cornerCoords [8];

            bbox.getCornerPoints(cornerCoords);

            for (int i = 0; i < 8; i++)
                corners[i] = gridPtr->transform().indexToWorld(cornerCoords[i].asVec3d());
        };

        int i = 0;

        if (frustumCulling)
        {
            openvdb::Vec3f corners [8];
            bboxCornersWorldSpace(gridPtr->evalActiveVoxelBoundingBox(), corners);
            // for (int i = 0; i < 8; i++)
                // cb((std::to_string(corners[i].x()) + ", " + std::to_string(corners[i].y()) + ", " + std::to_string(corners[i].z()) + ", ").c_str());

            if (!testIntersection(corners, worldToClipMatrix, cb))
            {
                cb("Culling all");
                return 0;
            }
        }

        // cb("Culling done");
        // Iterate over all active voxels
        // {
            // switch depth
            // case 0:
            //    if frustum culling
            //          check for intersection, if not, cull and exit loop
            //  if lod
            //      get voxelsize
            //      if voxelsize / dist < #, interpolate all points
            // if (lod) {
                // Check distance from voxel to camera and interpolate distant points
                // If accumulated, skip children
            // }
            
            // If child node reached, add to verts
        // }
        
        // Default -- no frustum culling
        for (auto leafIter = gridPtr->tree().cbeginLeaf(); leafIter; ++leafIter)
        {
            const AttributeArray &positionArray = leafIter->constAttributeArray("P");
            AttributeHandle<openvdb::Vec3f> positionHandle(positionArray);

            const AttributeArray &colorArray = leafIter->constAttributeArray("Cd");
            // AttributeHandle<openvdb::Vec3f> colorHandle(colorArray);
            AttributeHandle<int> colorHandle(colorArray);

            for (auto indexIter = leafIter->beginIndexOn(); indexIter; ++indexIter)
            {
                openvdb::Vec3f voxelPos = positionHandle.get(*indexIter);
                int col = colorHandle.get(*indexIter);

                openvdb::Vec3d xyz = indexIter.getCoord().asVec3d();
                // pa.add(grid->transform().indexToWorld(voxelPos + xyz), voxelSize);
                openvdb::Vec3d worldPos = gridPtr->transform().indexToWorld(voxelPos + xyz);

                Color col32 = hex2rgb(col);
                points[i] = { (float)worldPos.x(), (float)worldPos.y(), (float)worldPos.z(), (float)col32.r / 255.0f, (float)col32.g / 255.0f, (float)col32.b / 255.0f, 1.0f };
                // verts[i] = { (float)worldPos.x(), (float)worldPos.y(), (float)worldPos.z() };

                // string message = "Adding Vertex: " + to_string(result[i].x) + ", " + to_string(result[i].y) + ", " + to_string(result[i].z);
                // cb(message.c_str());

                i++;
            }
        }
        return i;

        }
};