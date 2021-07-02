#pragma once

#include <openvdb/openvdb.h>
#include <openvdb/points/PointConversion.h>

typedef void (*LoggingCallback)(const char *message);

struct Point
{
    float x, y, z, r, g, b, a;
};

struct Color
{
    uint8_t r, g, b, a;
};


bool testIntersection(openvdb::Vec3f *corners, openvdb::math::Mat4s cam, LoggingCallback cb)
{
    for (int i = 0; i < 8; i++)
    {
        openvdb::Vec3f c = corners[i];
        openvdb::Vec4f cWorldPos((float)c.x(), (float)c.y(), (float)c.z(), 1.0f);
        openvdb::Vec4f cClipPos = cam * cWorldPos;
        openvdb::Vec3f cClipPos3(cClipPos.x() / cClipPos.w(), cClipPos.y() / cClipPos.w(), cClipPos.z() / cClipPos.w());
        // cb(
        //     (std::to_string(c.x()) + ", " + std::to_string(c.y()) + ", " + std::to_string(c.z()) + " to " +
        //     std::to_string(cClipPos3.x()) + ", " + std::to_string(cClipPos3.y()) + ", " + std::to_string(cClipPos3.z())).c_str()
        // );
        if (openvdb::math::Abs(cClipPos3.x()) < 1 && openvdb::math::Abs(cClipPos3.y()) < 1 && openvdb::math::Abs(cClipPos3.z()) < 1)
            return true;
    }
}

int rgb2hex(openvdb::Vec3i rgb) {
    return (rgb.x() << 16) + (rgb.y() << 8) + rgb.z();
}

Color hex2rgb(int hex) {
    return { (uint8_t)((hex >> 16) & 0xFF), (uint8_t)((hex >> 8) & 0xFF), (uint8_t)(hex & 0xFF), (uint8_t) 255 };
}
