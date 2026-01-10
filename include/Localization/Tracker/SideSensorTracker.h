#pragma once
#ifndef SideSensorTracker_H
#define SideSensorTracker_H

#include "TrackerInterface.h"
#include "api.h"
#include "TrackerConfig.h"
#include "ConfiguredMotorGroup.h"

class SideSensorTracker : public TrackerInterface {
   public:
    SideSensorTracker(TrackerConfig &side_config, ConfiguredMotorGroup &left, ConfiguredMotorGroup &right) : 
        side_config(side_config), left_motors(left), right_motors(right) {};
    float getForwardDistance() override;
    float getSideDistance() override;
    float getFwdCenterDistance() override { return 0.0f; };
    float getSideCenterDistance() override { return side_config.center_distance; };

   private:
    TrackerConfig &side_config;
    ConfiguredMotorGroup &left_motors;
    ConfiguredMotorGroup &right_motors;
};

#endif  // SideSensorTracker_H