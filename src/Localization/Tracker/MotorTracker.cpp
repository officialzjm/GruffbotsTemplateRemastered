#include "MotorTracker.h"
#include "numeric"
#include <vector>

using namespace pros;

MotorTracker::MotorTracker(ConfiguredMotorGroup &left, ConfiguredMotorGroup &right):
    left_motors(left),
    right_motors(right)
{
}

float MotorTracker::getForwardDistance()
{
    left_motors.getMotorGroup().set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    right_motors.getMotorGroup().set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    std::vector<double> left_values = left_motors.getMotorGroup().get_position_all();
    float avg_left_deg = std::accumulate(left_values.begin(), left_values.end(), 0.0) / left_values.size();
    float avg_left_distance = (avg_left_deg / 360.0) * left_motors.getConfig().gear_ratio * left_motors.getConfig().wheel_diameter * M_PI;

    std::vector<double> right_values = right_motors.getMotorGroup().get_position_all();
    float avg_right_deg = std::accumulate(right_values.begin(), right_values.end(), 0.0) / right_values.size();
    float avg_right_distance = (avg_right_deg / 360.0) * right_motors.getConfig().gear_ratio * right_motors.getConfig().wheel_diameter * M_PI;

    return (avg_left_distance + avg_right_distance) / 2.0f;
}

float MotorTracker::getSideDistance()
{
    return 0.0f;
}
