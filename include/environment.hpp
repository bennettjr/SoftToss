#pragma once
#include "vec3.hpp"
#include <string>

namespace SoftToss
{

    struct Environment
    {
        float temp;  // Temperature
        float elev;  // Elevation
        float humid; // relative humidity
        float pres;  // barometric pressure

        float rho; // air density slug/ft^3

        float windSpeed;  // ft/s
        float windDir;    // degrees
        float windHeight; // ft
    };

    struct Contact
    {
        std::string type;
        Vec3 point;

        Contact(const std::string &t, const Vec3 &p) : type(t), point(p) {}
    };

}; // namespace SoftToss
