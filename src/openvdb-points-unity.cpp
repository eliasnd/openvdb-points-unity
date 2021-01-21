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
// TODO add quality modifier for createLevelSet
void computeMeshFromPointGrid(SharedPointDataGridReference *reference, size_t &pointCount, size_t &triCount, LoggingCallback cb)
{
    // https://github.com/AcademySoftwareFoundation/openvdb/blob/master/openvdb/viewer/RenderModules.cc (MeshOp)
    string message = "Constructing Mesh from Point Grid. This Could Take a While";
    cb(message.c_str());
    MyParticleList pa;
    PointDataGrid::Ptr grid = reference->gridPtr;
    // put a check in using hasUniformVoxels
    openvdb::Real voxelSize = grid->voxelSize().x();
    for (auto leafIter = grid->tree().cbeginLeaf(); leafIter; ++leafIter)
    {
        const AttributeArray &positionArray = leafIter->constAttributeArray("P");
        AttributeHandle<openvdb::Vec3f> positionHandle(positionArray);
        for (auto indexIter = leafIter->beginIndexOn(); indexIter; ++indexIter)
        {
            openvdb::Vec3f voxelPos = positionHandle.get(*indexIter);
            openvdb::Vec3d xyz = indexIter.getCoord().asVec3d();
            pa.add(grid->transform().indexToWorld(voxelPos + xyz), voxelSize);
        }
    }
    triCount = 0;
    pointCount = 0;
    openvdb::FloatGrid::Ptr floatGrid = openvdb::createLevelSet<openvdb::FloatGrid>(voxelSize / 2, voxelSize * 4);
    openvdb::tools::ParticlesToLevelSet<openvdb::FloatGrid> raster(*floatGrid);
    raster.setRmin(voxelSize);
    raster.setGrainSize(1);
    raster.rasterizeSpheres(pa);
    raster.finalize();
    floatGrid->setName("FloatGrid");
    openvdb::FloatGrid::Ptr sampled = downsampleGrid<openvdb::FloatGrid>(floatGrid, SampleQuality::High);
    openvdb::tools::VolumeToMesh mesher(0);
    mesher(*sampled);
    pointCount = mesher.pointListSize() * 3;
    triCount = 0;
    openvdb::tools::PolygonPoolList &polygonPoolList = mesher.polygonPoolList();
    for (openvdb::Index64 i = 0, j = mesher.polygonPoolListSize(); i < j; ++i)
    {
        triCount += polygonPoolList[i].numTriangles();
    }
    message = "Total Vertices: " + to_string(pointCount) + "\n" + "Total Faces: " + to_string(triCount);
    cb(message.c_str());
}

void destroySharedPointDataGridReference(SharedPointDataGridReference *reference)
{
    delete reference;
}

void generatePointArray(SharedPointDataGridReference *reference, float *points)
{
    auto tree = reference->gridPtr->tree();
    auto leafIter = tree.cbeginLeaf();

    for (int i = 0; i < tree.leafCount(); i++) {
        auto leaf = leafIter.getLeaf();
        points[3*i] = -1;
        points[3*i+1] = -1;
        points[3*i+2] = -1;
        ++leafIter;
    }
}
