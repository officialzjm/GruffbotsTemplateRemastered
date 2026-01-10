#include "FwdSensorTracker.h"

float FwdSensorTracker::getForwardDistance()
{
    float degrees = fwd_config.tracker_sensor.get_position()/100;
    return (degrees / 360.0f) * fwd_config.wheel_diameter * M_PI * fwd_config.gear_ratio;
}

float FwdSensorTracker::getSideDistance()
{
    return 0.0f;
}
