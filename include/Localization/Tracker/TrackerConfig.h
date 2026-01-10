#pragma once
#ifndef TrackerConfig_H
#define TrackerConfig_H

#include "api.h"

struct TrackerConfig {
    float wheel_diameter;  // in inches
    float gear_ratio;      // output speed / input speed
    float center_distance; // distance from robot center to tracking wheel in inches
    pros::Rotation &tracker_sensor;
};

#endif  // TrackerConfig_H