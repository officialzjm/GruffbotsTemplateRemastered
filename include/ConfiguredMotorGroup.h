#pragma once
#ifndef CONFIGUREDMOTORGROUP_H
#define CONFIGUREDMOTORGROUP_H

#include "api.h"
using namespace pros;

struct MotorConfig {
    /// @brief Wheel Diameter in inches
    /// @brief Gear Ratio (input speed / output speed)
    float wheel_diameter;  // in inches
    float gear_ratio;      // output speed / input speed
    float motor_rpm;
};

class ConfiguredMotorGroup {
    public:
        ConfiguredMotorGroup(MotorConfig config, MotorGroup& motors) : motor_config(config), motor_group(motors) {};
        MotorConfig getConfig() const { return motor_config; };
        MotorGroup& getMotorGroup() const { return motor_group; };
    private:
        MotorConfig motor_config;
        MotorGroup& motor_group;
};

#endif  // CONFIGUREDMOTORGROUP_H