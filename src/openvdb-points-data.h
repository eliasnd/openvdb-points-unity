#pragma once

#include <openvdb/openvdb.h>
#include <openvdb/points/PointConversion.h>
#include <openvdb/points/PointCount.h>
#include "utils.h"

using namespace std;
using namespace openvdb::points;

using RootType = openvdb::tree::RootNode<openvdb::tree::InternalNode<openvdb::tree::InternalNode <PointDataLeafNode<openvdb::PointDataIndex32, 3>, 4>, 5>>;
using InternalType1 = openvdb::tree::InternalNode<openvdb::tree::InternalNode <PointDataLeafNode<openvdb::PointDataIndex32, 3>, 4>, 5>;
using InternalType2 = openvdb::tree::InternalNode <PointDataLeafNode<openvdb::PointDataIndex32, 3>, 4>;
using LeafType = PointDataLeafNode<openvdb::PointDataIndex32, 3>;

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
            // cb(message.c_str());

            string grid(gridName);

            if (grid.compare("") == 0)
            {
                openvdb::io::File file(filename);
                file.open();

                grid = file.beginName().gridName();

                string message2 = "Loading Grid " + grid;
                // cb(message2.c_str());
            }

            openvdb::io::File fileHandle(filename);
            fileHandle.open();
            // cb("Trying gridptrcast");
            gridPtr = openvdb::gridPtrCast<PointDataGrid>(fileHandle.readGrid(gridName));
            fileHandle.close();
            // string msg2 = "Point count " + std::to_string(openvdb::points::pointCount(gridPtr->tree()));
            string msg2 = "Point count " + std::to_string(this->pointCount());
            // cb(msg2.c_str());
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

    Index32_3 treeShape()
    {
        std::vector<openvdb::Index32> countVec = gridPtr->tree().nodeCount();
        return { countVec[2], countVec[1], countVec[0] };
    };
    
    // NEW IDEA:
    // Millions of points too many to populate every frame
    // Just compute active voxels by checking each for frustum culling
    // If any voxel dirty, update buffer

    // openvdb::Index64 populatePoints(openvdb::math::Mat4s worldToClipMatrix, bool frustumCulling, bool lod, Point *points, LoggingCallback cb)
    openvdb::Index64 populatePoints(Point *points)//, LoggingCallback cb)
    {
        // return openvdb::points::pointCount(gridPtr->tree());
        int i = 0;
        // cb(("Populate point count " + std::to_string(this->pointCount())).c_str());
        
        // Default -- no frustum culling
        for (auto leafIter = gridPtr->tree().cbeginLeaf(); leafIter; ++leafIter)
        {
            const AttributeArray &positionArray = leafIter->constAttributeArray("P");
            AttributeHandle<openvdb::Vec3f> positionHandle(positionArray);

            const AttributeArray &colorArray = leafIter->constAttributeArray("Cd");
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

    void populateTreeOffsets(int *layer1Offsets, int *layer2Offsets, int *leafNodeOffsets)
    {
        int index1 = 0, index2 = 0, index3 = 0;

        for (PointDataTree::NodeIter iter = gridPtr->tree().beginNode(); iter; ++iter)
        {
            switch (iter.getDepth()) {
                case 0:
                    break;
                case 1:
                {
                    InternalType1 *node = nullptr;
                    iter.getNode(node);
                    layer1Offsets[index1] = index1 > 0 ? node->childCount() + layer1Offsets[index1-1] : node->childCount();
                    index1++;
                    break;
                }
                case 2:
                {
                    InternalType2 *node = nullptr;
                    iter.getNode(node);
                    layer2Offsets[index2] = index2 > 0 ? node->childCount() + layer2Offsets[index2-1] : node->childCount();
                    index2++;
                    break;
                }
                case 3:
                {
                    LeafType *node = nullptr;
                    iter.getNode(node);
                    leafNodeOffsets[index3] = index3 > 0 ? node->pointCount() + leafNodeOffsets[index3-1] : node->pointCount();
                    index3++;
                    break;
                }
            }
        }
    }

    /* 
    Populates mask of tree in 3 layers. For each entry in mask, which represent nodes,
        -1 = do not render
        0 = accumulate
        1 = render normally
    */
    void populateTreeMask(
        openvdb::math::Mat4s m, openvdb::math::Mat4s v, openvdb::math::Mat4s p,
        bool frustumCulling, bool lod, bool occlusionCulling, 
        int *internal1Mask, int *internal2Mask, int *leafNodeMask, 
        LoggingCallback cb
    )
    {
        // cb("Got mvp");
        // cb(mat4_to_string(cam).c_str());
        int index1 = 0, index2 = 0, index3 = 0;

        // BBox fields
        openvdb::Vec3d corners [8];
        openvdb::Vec3d center;
        float bboxSize;
        
        RootType root = gridPtr->tree().root();

        for (RootType::ChildOnIter internalIter1 = root.beginChildOn(); internalIter1; ++internalIter1)
        {
            // bboxCornersWorldSpace(internalIter1->getNodeBoundingBox(), corners, cb);
            processBBox(internalIter1->getNodeBoundingBox(), corners, &center, &bboxSize, cb);

            if (frustumCulling && !testIntersection(corners, p*v*m, cb, false))
            {
                // cb("No intersection");
                internal1Mask[index1] = -1;
                index2 += internalIter1->childCount();
            }
            else if (lod && testAccumulation(center, bboxSize, v*m, cb))
            {
                internal1Mask[index1] = 0;
                index2 += internalIter1->childCount();
            }
            else
            {
                // testAccumulation(center, bboxSize, v*m, cb);

                internal1Mask[index1] = 1;
                for (InternalType1::ChildOnIter internalIter2 = internalIter1->beginChildOn(); internalIter2; ++internalIter2)
                {
                    // bboxCornersWorldSpace(internalIter2->getNodeBoundingBox(), corners, cb);
                    processBBox(internalIter2->getNodeBoundingBox(), corners, &center, &bboxSize, cb);

                    if (frustumCulling && !testIntersection(corners, p*v*m, cb, false))
                    {
                        internal2Mask[index2] = -1;
                        index3 += internalIter2->childCount();
                    }
                    else if (lod && testAccumulation(center, bboxSize, v*m))
                    {
                        internal2Mask[index2] = 0;
                        index3 += internalIter1->childCount();
                    }
                    else
                    {
                        internal2Mask[index2] = 1;
                        for (InternalType2::ChildOnIter leafIter = internalIter2->beginChildOn(); leafIter; ++leafIter)
                        {
                            // bboxCornersWorldSpace(leafIter->getNodeBoundingBox(), corners, cb);
                            processBBox(leafIter->getNodeBoundingBox(), corners, &center, &bboxSize, cb);

                            if (frustumCulling && !testIntersection(corners, p*v*m, cb, false))
                                leafNodeMask[index3] = -1;
                            else if (testAccumulation(center, bboxSize, v*m))
                                leafNodeMask[index3] = 0;
                            else
                                leafNodeMask[index3] = 1;

                            index3++;
                        }
                    }

                    index2++;
                }
            }

            index1++;
        }
    }

    // Populates a linear array with accumulated points. Array should be laid out [[layer1], [layer2], [leafNodes]]
    void populateAccumulatedPoints(Point *points)
    {
        Index32_3 indices = treeShape();
        indices.z = indices.x + indices.y;
        indices.y = indices.x;
        indices.x = 0;

        RootType root = gridPtr->tree().root();

        for (RootType::ChildOnIter internalIter1 = root.beginChildOn(); internalIter1; ++internalIter1)
        {
            Point avgI1Point = {0, 0, 0, 0, 0, 0, 0};

            for (InternalType1::ChildOnIter internalIter2 = internalIter1->beginChildOn(); internalIter2; ++internalIter2)
            {
                Point avgI2Point = {0, 0, 0, 0, 0, 0, 0};

                for (InternalType2::ChildOnIter leafIter = internalIter2->beginChildOn(); leafIter; ++leafIter)
                {
                    Point avgLPoint = {0, 0, 0, 0, 0, 0, 0};                    

                    const AttributeArray &positionArray = leafIter->constAttributeArray("P");
                    AttributeHandle<openvdb::Vec3f> positionHandle(positionArray);

                    const AttributeArray &colorArray = leafIter->constAttributeArray("Cd");
                    AttributeHandle<int> colorHandle(colorArray);

                    for (auto indexIter = leafIter->beginIndexOn(); indexIter; ++indexIter)
                    {
                        openvdb::Vec3d worldPos = gridPtr->transform().indexToWorld(positionHandle.get(*indexIter) + indexIter.getCoord().asVec3d());
                        Color col32 = hex2rgb(colorHandle.get(*indexIter));

                        avgLPoint += { (float)worldPos.x(), (float)worldPos.y(), (float)worldPos.z(), (float)col32.r / 255.0f, (float)col32.g / 255.0f, (float)col32.b / 255.0f, 1.0f }; 

                    }

                    avgLPoint /= leafIter->pointCount();

                    points[indices.z] = avgLPoint;
                    indices.z++;

                    avgI2Point += avgLPoint;
                }

                avgI2Point /= internalIter2->childCount();
                points[indices.y] = avgI2Point;
                indices.y++;

                avgI1Point += avgI2Point;
            }

            avgI1Point /= internalIter1->childCount();

            points[indices.x] = avgI1Point;
            indices.x++;
        }
    }

private:
    void bboxCornersWorldSpace(openvdb::math::CoordBBox bbox, openvdb::Vec3d *corners, LoggingCallback cb)
    {
        openvdb::math::Coord cornerCoords [8];

        bbox.getCornerPoints(cornerCoords);

        for (int i = 0; i < 8; i++) {
            corners[i] = gridPtr->transform().indexToWorld(cornerCoords[i].asVec3d());
        }
    }

    void processBBox(openvdb::math::CoordBBox bbox, openvdb::Vec3d *corners, openvdb::Vec3d *center, float *size, LoggingCallback cb)
    {
        openvdb::math::Coord cornerCoords [8];

        bbox.getCornerPoints(cornerCoords);

        for (int i = 0; i < 8; i++) {
            corners[i] = gridPtr->transform().indexToWorld(cornerCoords[i].asVec3d());
        }

        *center = gridPtr->transform().indexToWorld(bbox.getCenter());
        
        openvdb::Vec3d bboxDiff;
        bboxDiff.sub(gridPtr->transform().indexToWorld(bbox.getEnd().asVec3d()), gridPtr->transform().indexToWorld(bbox.getStart().asVec3d()));
        *size = bboxDiff.length();
    }

    // Classifies bounding box as visible, occluded, or outside camera
    int classifyBBox(openvdb::math::CoordBBox bbox)
    {

    }
};