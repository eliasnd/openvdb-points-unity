#pragma once

#include <openvdb/openvdb.h>
#include <openvdb/points/PointConversion.h>

typedef void (*LoggingCallback)(const char *message);

struct Index32_3
{
    openvdb::Index32 x, y, z;
};

struct Point
{
    float x, y, z, r, g, b, a;
};

struct Color
{
    uint8_t r, g, b, a;
};

template <typename T>
string vec3_to_string(openvdb::math::Vec3<T> vec)
{
    return "[ " + std::to_string(vec.x()) + ", " + std::to_string(vec.y()) + ", " + std::to_string(vec.z()) + " ]";
}

template <typename T>
string vec4_to_string(openvdb::math::Vec4<T> vec)
{
    return "[ " + std::to_string(vec.x()) + ", " + std::to_string(vec.y()) + ", " + std::to_string(vec.z()) + ", " + std::to_string(vec.w()) + " ]";
}

template <typename T>
string mat4_to_string(openvdb::math::Mat4<T> mat)
{
    string result = "{";
    for (int i = 0; i < 4; i++)
        result += "\n  [" + std::to_string(mat.row(i).x()) + ", " + std::to_string(mat.row(i).y()) + ", " + std::to_string(mat.row(i).z()) + ", " + std::to_string(mat.row(i).w()) + " ]";
    result += "}";
    return result;
}


// NOTE: What if box surrounds frusum? Have to check what side corners OOB on
bool testIntersection(openvdb::Vec3d *corners, openvdb::math::Mat4s cam, LoggingCallback cb, bool debug)
// bool testIntersection(openvdb::Vec3d *corners, openvdb::math::Mat4s cam)
{
    if (debug)
    {
        cb((
            "Testing intersection of corners " + 
            vec3_to_string(corners[0]) + ",\n" + 
            vec3_to_string(corners[1]) + ",\n" +
            vec3_to_string(corners[2]) + ",\n" + 
            vec3_to_string(corners[3]) + ",\n" + 
            vec3_to_string(corners[4]) + ",\n" + 
            vec3_to_string(corners[5]) + ",\n" + 
            vec3_to_string(corners[6]) + ",\n" + 
            vec3_to_string(corners[7]) + "." 
        ).c_str());
        cb("Matrix: ");
        cb(mat4_to_string(cam).c_str());
    }


    openvdb::Vec3f clipCorners [8];

    for (int i = 0; i < 8; i++)
    {
        openvdb::Vec3d c = corners[i];
        if (debug)
            cb(("Starting coord: " + vec3_to_string(c)).c_str());
        openvdb::Vec4f cWorldPos((float)c.x(), (float)c.y(), (float)c.z(), 1.0f);
        if (debug)
            cb(("Homogeneous coord: " + vec4_to_string(cWorldPos)).c_str());
        openvdb::Vec4f cClipPos = cam * cWorldPos;
        if (debug)
            cb(("Homogeneous clip: " + vec4_to_string(cClipPos)).c_str());
        openvdb::Vec3f cClipPos3(cClipPos.x() / cClipPos.w(), cClipPos.y() / cClipPos.w(), cClipPos.z() / cClipPos.w());
        if (debug)
            cb(("Normalized clip: " + vec3_to_string(cClipPos3)).c_str());
        // How is clip pos a v3??
        if (openvdb::math::Abs(cClipPos3.x()) < 1 && openvdb::math::Abs(cClipPos3.y()) < 1 && openvdb::math::Abs(cClipPos3.z()) < 1)
            return true;

        clipCorners[i] = cClipPos3;
    }

    // If bounding box corners don't intersect frusum, check if they all fall outside on different sides of frustum
    // i.e. bbox surrounds frustum and should be rendered

    bool xAgree = true, yAgree = true, zAgree = true;

    if (debug)
        cb(("Corner " + vec3_to_string(clipCorners[0]) + ", agreement: " + std::to_string(xAgree) + ", " + std::to_string(yAgree) + ", " + std::to_string(zAgree)).c_str());

    for (int i = 1; i < 8; i++)
    {
        xAgree &= (clipCorners[i].x() * clipCorners[i-1].x() > 0);
        yAgree &= (clipCorners[i].y() * clipCorners[i-1].y() > 0);
        zAgree &= (clipCorners[i].z() * clipCorners[i-1].z() > 0);

        if (debug)
            cb(("Corner " + vec3_to_string(clipCorners[i]) + ", agreement: " + std::to_string(xAgree) + ", " + std::to_string(yAgree) + ", " + std::to_string(zAgree)).c_str());

        if (!(xAgree || yAgree))// || zAgree))
            return true;
    }

    return false;
}

// Test if bbox should be accumulated for lod given extents  
bool testAccumulation(openvdb::Vec3d bboxCenter, float bboxSize, openvdb::math::Mat4s mv, LoggingCallback cb=nullptr)
{
    if (cb != nullptr)
        cb(("Testing accumulation. Center is " + vec3_to_string(bboxCenter) + " and size is " + std::to_string(bboxSize) + ".").c_str());

    float cutoff = 0.05; // Maximum distance allowed between max and min in clip space before accumulation

    openvdb::Vec4f hBBoxCenter((float)bboxCenter.x(), (float)bboxCenter.y(), (float)bboxCenter.z(), 1);
    openvdb::Vec4f hBBoxCenterView = mv * hBBoxCenter;  // Only use mv to test depth from camera
    openvdb::Vec3f bboxCenterView(hBBoxCenterView.x() / hBBoxCenterView.w(), hBBoxCenterView.y() / hBBoxCenterView.w(), hBBoxCenterView.z() / hBBoxCenterView.w());

    if (cb != nullptr)
    {
        cb(("Bbox center in camera space is " + vec3_to_string(bboxCenterView)).c_str());
        cb(("Ratio is " + std::to_string(bboxSize / -bboxCenterView.z())).c_str());
    }
    
    return bboxSize / -bboxCenterView.z() < cutoff;     // Get ratio of size to distance from camera
}

// Tests whether bbox1 occludes bbox2 -- returns 1 if true, 0 if no occlusion, -1 if bbox2 occludes bbox1
int testOcclusion(openvdb::Vec3d *bbox1, openvdb::Vec3d *bbox2, openvdb::math::Mat4s cam, LoggingCallback cb = nullptr)
{
    return 0;
}


int rgb2hex(openvdb::Vec3i rgb) {
    return (rgb.x() << 16) + (rgb.y() << 8) + rgb.z();
}

Color hex2rgb(int hex) {
    return { (uint8_t)((hex >> 16) & 0xFF), (uint8_t)((hex >> 8) & 0xFF), (uint8_t)(hex & 0xFF), (uint8_t) 255 };
}

