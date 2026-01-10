#include "SideSensorTracker.h"
#include "numeric"

float SideSensorTracker::getForwardDistance()
{
    std::vector<double> left_values = left_motors.getMotorGroup().get_position_all();
    float avg_left_deg = std::accumulate(left_values.begin(), left_values.end(), 0.0) / left_values.size();
    float avg_left_distance = (avg_left_deg / 360.0) * left_motors.getConfig().wheel_diameter * M_PI;

    std::vector<double> right_values = right_motors.getMotorGroup().get_position_all();
    float avg_right_deg = std::accumulate(right_values.begin(), right_values.end(), 0.0) / right_values.size();
    float avg_right_distance = (avg_right_deg / 360.0) * right_motors.getConfig().wheel_diameter * M_PI;

    return (avg_left_distance + avg_right_distance) / 2.0f;
}

float SideSensorTracker::getSideDistance()
{
    float degrees = side_config.tracker_sensor.get_position()/100;
    return (degrees / 360.0f) * side_config.wheel_diameter * M_PI * side_config.gear_ratio;
}
