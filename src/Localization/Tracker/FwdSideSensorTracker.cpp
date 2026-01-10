#include "FwdSideSensorTracker.h"

float FwdSideSensorTracker::getForwardDistance()
{
    float degrees = fwd_config.tracker_sensor.get_position()/100;
    return (degrees / 360.0f) * fwd_config.wheel_diameter * M_PI * fwd_config.gear_ratio;
}

float FwdSideSensorTracker::getSideDistance()
{
    float degrees = side_config.tracker_sensor.get_position()/100;
    return (degrees / 360.0f) * side_config.wheel_diameter * M_PI * side_config.gear_ratio;
}
