// Heavily inspired by http://xdpixel.com/native-rendering-plugin-in-unity/

#include "openvdb-points-interface.h"

using RootType = openvdb::tree::RootNode<openvdb::tree::InternalNode<openvdb::tree::InternalNode <PointDataLeafNode<openvdb::PointDataIndex32, 3>, 4>, 5>>;
using InternalType1 = openvdb::tree::InternalNode<openvdb::tree::InternalNode <PointDataLeafNode<openvdb::PointDataIndex32, 3>, 4>, 5>;
using InternalType2 = openvdb::tree::InternalNode <PointDataLeafNode<openvdb::PointDataIndex32, 3>, 4>;
using LeafType = PointDataLeafNode<openvdb::PointDataIndex32, 3>;

int populatePoints(OpenVDBPointsData *data, Point *points)
{
    return (int)data->populatePoints(points);
}

// OpenVDB Functionality

void openvdbInitialize()
{
    openvdb::initialize();
}

void openvdbUninitialize()
{
    openvdb::uninitialize();
}

bool convertPLYToVDB(const char *filename, const char *outfile, LoggingCallback cb)
{
    try
    {
        string filePath(filename);
        string outPath(outfile);
        string message = "Converting " + filePath + "to VDB format";
        cb(message.c_str());
        PLYReader::PointData<float, uint8_t> cloud = PLYReader::readply(filePath);
        cloudToVDB(cloud, outPath);
        message = "Successfully converted " + filePath + " to " + outPath;
        cb(message.c_str());
        return true;
    }
    catch (exception &e)
    {
        cerr << "Error: " << e.what() << endl;
        cb(e.what());
        return false;
    }
}

void cloudToVDB(PLYReader::PointData<float, uint8_t> cloud, string filename)
{
    try
    {
        if (cloud.vertices.size() == 0)
            throw;
        vector<openvdb::Vec3R> positions;

        for (vector<PLYReader::point<float>>::iterator it = cloud.vertices.begin(); it != cloud.vertices.end(); it++)
        {
            positions.push_back(openvdb::Vec3R(it->x, it->y, it->z));
        }

        if (positions.size() == 0)
            throw;

        PointAttributeVector<openvdb::Vec3R> positionsWrapper(positions);
        int pointsPerVoxel = 8;

        float voxelSize = computeVoxelSize(positionsWrapper, pointsPerVoxel);
        openvdb::math::Transform::Ptr transform = openvdb::math::Transform::createLinearTransform(voxelSize);
        openvdb::tools::PointIndexGrid::Ptr pointIndex = openvdb::tools::createPointIndexGrid<openvdb::tools::PointIndexGrid>(positionsWrapper, *transform);
        // no compression
        PointDataGrid::Ptr grid = createPointDataGrid<FixedPointCodec<false>,
                                                      PointDataGrid>(*pointIndex, positionsWrapper, *transform);

        grid->setName("Points");

        positions.clear(); // Should be alloced and populated in grid directly?

        // handle color
        // based on https://github.com/AcademySoftwareFoundation/openvdb/blob/f44e305f8c3181d0cbf667fe5da0510f378b9256/openvdb_houdini/houdini/VRAY_OpenVDB_Points.cc
        if (cloud.color.size() > 0)
        {
            vector<int> colors;
            for (vector<PLYReader::rgb<uint8_t>>::iterator it = cloud.color.begin(); it != cloud.color.end(); it++)
            {
                colors.push_back(rgb2hex(openvdb::Vec3i(it->r, it->g, it->b)));
            }
            PointDataTree &tree = grid->tree();
            openvdb::tools::PointIndexTree &pointIndexTree = pointIndex->tree();

            appendAttribute<int>(tree, "Cd");
            PointAttributeVector<int> colorWrapper(colors);
            populateAttribute<PointDataTree, openvdb::tools::PointIndexTree, PointAttributeVector<int>>(tree, pointIndexTree, "Cd", colorWrapper);
        }

        // cb("Writing to file");

        // Wrte the file
        openvdb::io::File outfile(filename);
        openvdb::GridPtrVec grids;
        grids.push_back(grid);
        outfile.write(grids);
        outfile.close();
        cout << "Successfully saved VDB to " << filename << endl;
    }
    catch (...)
    {
        cout << "Something went wrong!" << endl;
    }
}

OpenVDBPointsData *readPointDataFromFile(const char *filename, const char *gridName, LoggingCallback cb)
{
    OpenVDBPointsData *data = new OpenVDBPointsData(filename, gridName, cb);
    return data;
}

Index32_3 getTreeShape(OpenVDBPointsData *data)
{
    return data->treeShape();
}

openvdb::Index64 getPointCountFromGrid(OpenVDBPointsData *reference)
{
    return reference->pointCount();
}

void populateTreeOffsets(OpenVDBPointsData *data, int *layer1Offsets, int *layer2Offsets, int *leafNodeOffsets)
{
    data->populateTreeOffsets(layer1Offsets, layer2Offsets, leafNodeOffsets);
}

void populateTreeMask(OpenVDBPointsData *data, openvdb::math::Mat4s m, openvdb::math::Mat4s v, openvdb::math::Mat4s p, bool frustumCulling, bool lod, bool occlusionCulling, int *internal1Mask, int *internal2Mask, int *leafNodeMask, LoggingCallback cb)
{
    data->populateTreeMask(m, v, p, frustumCulling, lod, occlusionCulling, internal1Mask, internal2Mask, leafNodeMask, cb);
}

void populateAccumulatedPoints(OpenVDBPointsData *data, Point *points)
{
    data->populateAccumulatedPoints(points);
}

void destroyPointData(OpenVDBPointsData *reference)
{
    delete reference;
}