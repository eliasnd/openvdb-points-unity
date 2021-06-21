#include <iostream>
#include "openvdb-points-unity.h"

using namespace std;
using namespace openvdb::points;

void openvdbInitialize()
{
    openvdb::initialize();
}

void openvdbUninitialize()
{
    openvdb::uninitialize();
}

int rgb2hex(openvdb::Vec3i rgb) {
    return (rgb.x() << 16) + (rgb.y() << 8) + rgb.z();
}

Color hex2rgb(int hex) {
    return { (uint8_t)((hex >> 16) & 0xFF), (uint8_t)((hex >> 8) & 0xFF), (uint8_t)(hex & 0xFF), (uint8_t) 255 };
}

void cloudToVDB(PLYReader::PointData<float, uint8_t> cloud, string filename, LoggingCallback cb)
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
            // vector<openvdb::Vec3f> colors;
            vector<int> colors;
            for (vector<PLYReader::rgb<uint8_t>>::iterator it = cloud.color.begin(); it != cloud.color.end(); it++)
            {
                colors.push_back(rgb2hex(openvdb::Vec3i(it->r, it->g, it->b)));
            }
            PointDataTree &tree = grid->tree();
            openvdb::tools::PointIndexTree &pointIndexTree = pointIndex->tree();

            // appendAttribute<int, FixedPointCodec<false, UnitRange>>(tree, "Cd");
            appendAttribute<int>(tree, "Cd");
            // PointAttributeVector<openvdb::Vec3f> colorWrapper(colors);
            PointAttributeVector<int> colorWrapper(colors);
            // populateAttribute<PointDataTree, openvdb::tools::PointIndexTree, PointAttributeVector<openvdb::Vec3f>>(tree, pointIndexTree, "Cd", colorWrapper);
            populateAttribute<PointDataTree, openvdb::tools::PointIndexTree, PointAttributeVector<int>>(tree, pointIndexTree, "Cd", colorWrapper);
        }

        cb("Writing to file");

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
        // TODO improve this logging
        cb("Something went wrong");
        cout << "Something went wrong!" << endl;
    }
}

PointDataGrid::Ptr loadPointGrid(string filename, string gridName, LoggingCallback cb)
{
    openvdb::io::File fileHandle(filename);
    fileHandle.open();
    PointDataGrid::Ptr grid = openvdb::gridPtrCast<PointDataGrid>(fileHandle.readGrid(gridName));
    fileHandle.close();
    return grid;
}

template <typename GridType>
typename GridType::Ptr downsampleGrid(typename GridType::Ptr inGrid, SampleQuality quality)
{
    typename GridType::Ptr sampled = GridType::create();
    sampled->setTransform(openvdb::math::Transform::createLinearTransform(0.6 * quality));
    const openvdb::math::Transform &inTransform = inGrid->transform(),
                                   &outTransform = sampled->transform();
    openvdb::Mat4R xform = inTransform.baseMap()->getAffineMap()->getMat4() *
                           outTransform.baseMap()->getAffineMap()->getMat4().inverse();
    openvdb::tools::GridTransformer transformer(xform);
    transformer.transformGrid<openvdb::tools::QuadraticSampler, GridType>(*inGrid, *sampled);
    sampled->tree().prune();
    return sampled;
}

// Functions with C linkage

bool convertPLYToVDB(const char *filename, const char *outfile, LoggingCallback cb)
{
    try
    {
        string filePath(filename);
        string outPath(outfile);
        string message = "Converting " + filePath + "to VDB format";
        cb(message.c_str());
        PLYReader::PointData<float, uint8_t> cloud = PLYReader::readply(filePath);
        cloudToVDB(cloud, outPath, cb);
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

SharedPointDataGridReference *readPointGridFromFile(const char *filename, const char *gridName, LoggingCallback cb)
{
    SharedPointDataGridReference *reference = new SharedPointDataGridReference();
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

        reference->gridPtr = loadPointGrid(filePath, grid, cb);
        
    }
    catch (exception &e)
    {
        cb(e.what());
    }
    return reference;
}

openvdb::Index64 getPointCountFromGrid(SharedPointDataGridReference *reference)
{
    openvdb::Index64 count = pointCount(reference->gridPtr->tree());
    return count;
}

void populatePointArraysFromPointGrid(Pos *posArr, Color *colArr, SharedPointDataGridReference *reference, LoggingCallback cb)
{
    // MyParticleList pa;
    PointDataGrid::Ptr grid = reference->gridPtr;
    openvdb::Real voxelSize = grid->voxelSize().x();

    // Point *result = (Point*)malloc(sizeof(Point) * getPointCountFromGrid(reference)); // Is this awful?
    int i = 0;

    // cb(to_string(sizeof(Point)).c_str());

    for (auto leafIter = grid->tree().cbeginLeaf(); leafIter; ++leafIter)
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
            openvdb::Vec3d worldPos = grid->transform().indexToWorld(voxelPos + xyz);

            posArr[i] = { (float)worldPos.x(), (float)worldPos.y(), (float)worldPos.z() };
            colArr[i] = hex2rgb(col);

            // string message = "Adding Vertex: " + to_string(result[i].x) + ", " + to_string(result[i].y) + ", " + to_string(result[i].z);
            // cb(message.c_str());

            i++;
        }
    }
}

unsigned int populateVertices(SharedPointDataGridReference *reference, openvdb::math::Mat4s camTransform, Vertex *verts, LoggingCallback cb)
{
    PointDataGrid::Ptr grid = reference->gridPtr;

    // bool hasColorAttribute = grid->tree().cbeginLeaf()->attributeSet().find("Cd") != openvdb::points::AttributeSet::INVALID_POS;
    bool frustumCulling = !camTransform.isZero();   // Use zero matrix to signal no culling
    /* string message1 = "[ " + to_string(camTransform(0, 0)) + ", " + to_string(camTransform(0, 1)) + ", " + to_string(camTransform(0, 2)) + ", " + to_string(camTransform(0, 3)) + "]\n" +
                      "[ " + to_string(camTransform(1, 0)) + ", " + to_string(camTransform(1, 1)) + ", " + to_string(camTransform(1, 2)) + ", " + to_string(camTransform(1, 3)) + "]\n" +
                      "[ " + to_string(camTransform(2, 0)) + ", " + to_string(camTransform(2, 1)) + ", " + to_string(camTransform(2, 2)) + ", " + to_string(camTransform(2, 3)) + "]\n" +
                      "[ " + to_string(camTransform(3, 0)) + ", " + to_string(camTransform(3, 1)) + ", " + to_string(camTransform(3, 2)) + ", " + to_string(camTransform(3, 3)) + "]";
    cb(message1.c_str()); */

    unsigned int i = 0;

    for (auto leafIter = grid->tree().cbeginLeaf(); leafIter; ++leafIter)
    {
        const AttributeArray &positionArray = leafIter->constAttributeArray("P");
        AttributeHandle<openvdb::Vec3f> positionHandle(positionArray);

        /* const AttributeArray &colorArray;
        AttributeHandle<openvdb::Vec3f> colorHandle;
        if (hasColorAttribute)
        {
            colorArray = leafIter->constAttributeArray("Cd");
            colorArray = new AttributeHandle<openvdb::Vec3f>(colorArray);
        } */

        for (auto indexIter = leafIter->beginIndexOn(); indexIter; ++indexIter)
        {
            openvdb::Vec3f voxelPos = positionHandle.get(*indexIter);
            openvdb::Vec3d xyz = indexIter.getCoord().asVec3d();
            openvdb::Vec3d worldPos = grid->transform().indexToWorld(voxelPos + xyz);

            if (frustumCulling) 
            {
                openvdb::Vec4f hWorldPos((float)worldPos.x(), (float)worldPos.y(), (float)worldPos.z(), 1.0f);
                openvdb::Vec4f hClipPos = hWorldPos * camTransform;
                openvdb::Vec3f clipPos(hClipPos.x() / hClipPos.w(), hClipPos.y() / hClipPos.w(), hClipPos.z() / hClipPos.w());

                if (openvdb::math::Abs(clipPos.x()) < 1 && openvdb::math::Abs(clipPos.y()) < 1 && openvdb::math::Abs(clipPos.z()) < 1)
                {
                    verts[i] = {(float)worldPos.x(), (float)worldPos.y(), (float)worldPos.z(), (uint8_t)255, (uint8_t)255, (uint8_t)255, (uint8_t)255};

                    /* if (hasColorAttribute)
                {
                    openvdb::Vec3f color = colorHandle.get(*indexIter);
                    verts[i].r = (uint8_t)(255 * color.x());
                    verts[i].g = (uint8_t)(255 * color.y());
                    verts[i].b = (uint8_t)(255 * color.z());
                } */

                    i++;
                }
            }
            else
            {
                verts[i] = {(float)worldPos.x(), (float)worldPos.y(), (float)worldPos.z(), (uint8_t)255, (uint8_t)255, (uint8_t)255, (uint8_t)255};

                i++;
            }
            
        }
    }

    return i;
}

void destroySharedPointDataGridReference(SharedPointDataGridReference *reference)
{
    delete reference;
}
