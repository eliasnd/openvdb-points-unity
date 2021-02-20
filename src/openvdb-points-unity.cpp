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
        PointDataGrid::Ptr grid = createPointDataGrid<NullCodec,
                                                      PointDataGrid>(*pointIndex, positionsWrapper, *transform);

        grid->setName("Points");

        // handle color
        // based on https://github.com/AcademySoftwareFoundation/openvdb/blob/f44e305f8c3181d0cbf667fe5da0510f378b9256/openvdb_houdini/houdini/VRAY_OpenVDB_Points.cc
        if (cloud.color.size() > 0)
        {
            vector<openvdb::Vec3f> colors;
            for (vector<PLYReader::rgb<uint8_t>>::iterator it = cloud.color.begin(); it != cloud.color.end(); it++)
            {
                colors.push_back(openvdb::Vec3f((float)it->r / 255.0f, (float)it->g / 255.0f, (float)it->b / 255.0f));
            }
            PointDataTree &tree = grid->tree();
            openvdb::tools::PointIndexTree &pointIndexTree = pointIndex->tree();
            appendAttribute<openvdb::Vec3f, FixedPointCodec<false, UnitRange>>(tree, "Cd");
            PointAttributeVector<openvdb::Vec3f> colorWrapper(colors);
            populateAttribute<PointDataTree, openvdb::tools::PointIndexTree, PointAttributeVector<openvdb::Vec3f>>(tree, pointIndexTree, "Cd", colorWrapper);
        }
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
        cout << "Something went wrong!" << endl;
    }
}



PointDataGrid::Ptr loadPointGrid(string filename, string gridName, LoggingCallback cb)
{
    openvdb::io::File fileHandle(filename);
    fileHandle.open();
    PointDataGrid::Ptr grid = openvdb::gridPtrCast<PointDataGrid>(fileHandle.readGrid(gridName));

    if (grid == NULL) {
        string message = "Error: No grid named " + gridName;
        cb(message.c_str());
    }
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
        }

        string message2 = "Loading Grid " + grid;
        cb(message2.c_str());

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
    try
    {
        openvdb::Index64 count = pointCount(reference->gridPtr->tree());
        return count;
    }
    catch (exception &e)
    { }
    
}

Vec3d *generatePointArrayFromPointGrid(SharedPointDataGridReference *reference, LoggingCallback cb)
{
    PointDataGrid::Ptr grid = reference->gridPtr;

    Vec3d *result = (Vec3d*)malloc(sizeof(Vec3d) * getPointCountFromGrid(reference)); // Is this awful?
    int i = 0;

    // cb(to_string(sizeof(Point)).c_str());

    for (auto leafIter = grid->tree().cbeginLeaf(); leafIter; ++leafIter)
    {
        const AttributeArray &positionArray = leafIter->constAttributeArray("P");
        AttributeHandle<openvdb::Vec3f> positionHandle(positionArray);

        for (auto indexIter = leafIter->beginIndexOn(); indexIter; ++indexIter)
        {
            openvdb::Vec3f voxelPos = positionHandle.get(*indexIter);
            openvdb::Vec3d xyz = indexIter.getCoord().asVec3d();
            // pa.add(grid->transform().indexToWorld(voxelPos + xyz), voxelSize);
            openvdb::Vec3d worldPos = grid->transform().indexToWorld(voxelPos + xyz);

            result[i] = { worldPos.x(), worldPos.y(), worldPos.z() };

            // string message = "Adding Vertex: " + to_string(result[i].x) + ", " + to_string(result[i].y) + ", " + to_string(result[i].z);
            // cb(message.c_str());

            i++;
        }
    }

    return result;
}

// Stupid float to double casting but probably worth it to avoid second struct
Vec3d *generateColorArrayFromPointGrid(SharedPointDataGridReference *reference)
{
    PointDataGrid::Ptr grid = reference->gridPtr;

    Vec3d *result = (Vec3d *)malloc(sizeof(Vec3d) * getPointCountFromGrid(reference));
    int i = 0;


    for (auto leafIter = grid->tree().cbeginLeaf(); leafIter; ++leafIter)
    {
        const AttributeArray &colorArray = leafIter->constAttributeArray("Cd");
        AttributeHandle<openvdb::Vec3f> colorHandle(colorArray);

        for (auto indexIter = leafIter->beginIndexOn(); indexIter; ++indexIter)
        {
            openvdb::Vec3f color = colorHandle.get(*indexIter);

            result[i] = {(double)color.x(), (double)color.y(), (double)color.z()};

            i++;
        }
    }

    return result;
}

SharedPointDataGridReference *arraysToPointGrid(Vec3d *positionArr, Vec3d *colorArr, uint count)
{
    vector<openvdb::Vec3R> positions;
    vector<openvdb::Vec3f> colors;

    for (uint i = 0; i < count; i++) 
    {
        positions.push_back(openvdb::Vec3R(positionArr[i].x, positionArr[i].y, positionArr[i].z));
        colors.push_back(openvdb::Vec3f((float)colorArr[i].x, (float)colorArr[i].y, (float)colorArr[i].z));
    }

    if (positions.size() == 0)
        throw;

    PointAttributeVector<openvdb::Vec3R> positionsWrapper(positions);
    int pointsPerVoxel = 8;

    float voxelSize = computeVoxelSize(positionsWrapper, pointsPerVoxel);
    openvdb::math::Transform::Ptr transform = openvdb::math::Transform::createLinearTransform(voxelSize);
    openvdb::tools::PointIndexGrid::Ptr pointIndex = openvdb::tools::createPointIndexGrid<openvdb::tools::PointIndexGrid>(positionsWrapper, *transform);

    PointDataGrid::Ptr grid = createPointDataGrid<NullCodec, PointDataGrid>(*pointIndex, positionsWrapper, *transform);

    grid->setName("Points");

    PointDataTree &tree = grid->tree();
    openvdb::tools::PointIndexTree &pointIndexTree = pointIndex->tree();
    appendAttribute<openvdb::Vec3f, FixedPointCodec<false, UnitRange>>(tree, "Cd");
    PointAttributeVector<openvdb::Vec3f> colorWrapper(colors);
    populateAttribute<PointDataTree, openvdb::tools::PointIndexTree, PointAttributeVector<openvdb::Vec3f>>(tree, pointIndexTree, "Cd", colorWrapper);

    SharedPointDataGridReference *reference = new SharedPointDataGridReference();
    reference->gridPtr = grid;
    return reference;
}

void destroySharedPointDataGridReference(SharedPointDataGridReference * reference)
{
    delete reference;
}

// From https://people.cs.clemson.edu/~jtessen/cpsc8190/OpenVDB-dpawiki.pdf
openvdb::math::NonlinearFrustumMap *mapFromFrustum(Frustum frustum, openvdb::math::Vec3d voxelSize)
{
    return new openvdb::math::NonlinearFrustumMap(
        openvdb::Vec3d(frustum.pos.x, frustum.pos.y, frustum.pos.z),
        openvdb::Vec3d(frustum.dir.x, frustum.dir.y, frustum.dir.z),
        openvdb::Vec3d(frustum.up.x, frustum.up.y, frustum.up.z), // What is up???
        frustum.aspect,
        frustum.nPlane,
        frustum.fPlane,
        openvdb::math::floatToInt32((float)(frustum.nearWorldX / voxelSize.x())),
        openvdb::math::floatToInt32((float)((frustum.fPlane - frustum.nPlane) / voxelSize.z())));
}

SharedPointDataGridReference *occlusionMask(SharedPointDataGridReference *reference, Frustum frustum)
{
    openvdb::math::NonlinearFrustumMap *fMap = mapFromFrustum(frustum, reference->gridPtr->voxelSize());
    openvdb::points::PointDataGrid::Ptr clipped = openvdb::tools::clip(*reference->gridPtr, *fMap);

    return new SharedPointDataGridReference(clipped);
}
