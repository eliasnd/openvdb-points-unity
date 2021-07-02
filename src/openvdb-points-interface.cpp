// Heavily inspired by http://xdpixel.com/native-rendering-plugin-in-unity/

#include "openvdb-points-interface.h"

void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API UnityPluginLoad(IUnityInterfaces* unityInterfaces)
{
    // s_UnityInterfaces = unityInterfaces;
    // s_Graphics = unityInterfaces->Get<IUnityGraphics>();

    // s_Graphics->RegisterDeviceEventCallback(OnGraphicsDeviceEvent);

    // OnGraphicsDeviceEvent(kUnityGfxDeviceEventInitialize);
}

void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API UnityPluginUnload()
{
    // s_Graphics->UnregisterDeviceEventCallback(OnGraphicsDeviceEvent); 
}

static void UNITY_INTERFACE_API OnRenderEvent(int eventID, void *data)
{
    RenderingData rData;
    memcpy(&rData, data, sizeof(rData));

    rData.dataPtr->populatePoints(
        rData.worldToClipMatrix, 
        rData.frutumCulling, 
        rData.lod, 
        rData.pointsPtr, 
        rData.cb
    );
}

UnityRenderingEventAndData UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API GetRenderEventFunc()
{
    return OnRenderEvent;
}

void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API SetupResources(void *pointArray)
{
    // Set openvdb data to write to point array
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

            for (openvdb::Index32 i : tree.nodeCount())
                cb(std::to_string(i).c_str());

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

OpenVDBPointsData *readPointDataFromFile(const char *filename, const char *gridName, LoggingCallback cb)
{
    OpenVDBPointsData *data = new OpenVDBPointsData(filename, gridName, cb);
    return data;
    /* const void* address = static_cast<const void*>(&data);
    std::stringstream ss;
    ss << address;
    cb(ss.str().c_str());
    return &data; */
}

openvdb::Index64 getPointCountFromGrid(OpenVDBPointsData *reference)
{
    openvdb::Index64 count = pointCount(reference->gridPtr->tree());
    return count;
}

void destroyPointData(OpenVDBPointsData *reference)
{
    delete reference;
}