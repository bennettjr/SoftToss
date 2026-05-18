#pragma once
#include "vec3.hpp"
#include <string>

namespace SoftToss
{

    struct Environment
    {
        float gravity; // ft/s^2

        float temp;  // Temperature
        float elev;  // Elevation
        float humid; // relative humidity
        float pres;  // barometric pressure

        float rho; // air density slug/ft^3

        float windSpeed;  // ft/s
        float windDir;    // degrees
        float windHeight; // ft
    };

}; // namespace SoftToss
