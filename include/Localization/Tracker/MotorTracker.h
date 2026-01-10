#pragma once
#ifndef MOTORTRACKER_H
#define MOTORTRACKER_H

#include "TrackerInterface.h"
#include "api.h"
#include "ConfiguredMotorGroup.h"

class MotorTracker : public TrackerInterface {
    public:
        MotorTracker(ConfiguredMotorGroup &left, ConfiguredMotorGroup &right);
        float getForwardDistance() override;
        float getSideDistance() override;
        float getFwdCenterDistance() override { return 0.0f; };
        float getSideCenterDistance() override { return 0.0f; };

    private:
        ConfiguredMotorGroup& left_motors;
        ConfiguredMotorGroup& right_motors;
};

#endif  // MOTORTRACKER_H