#pragma once
#ifndef FwdSideSensorTracker_H
#define FwdSideSensorTracker_H

#include "TrackerInterface.h"
#include "api.h"
#include "TrackerConfig.h"

class FwdSideSensorTracker : public TrackerInterface {
   public:
    FwdSideSensorTracker(TrackerConfig &fwd_config, TrackerConfig &side_config)
        : fwd_config(fwd_config), side_config(side_config) {};
    float getForwardDistance() override;
    float getSideDistance() override;
    float getFwdCenterDistance() override { return fwd_config.center_distance; };
    float getSideCenterDistance() override { return side_config.center_distance; };

   private:
    TrackerConfig &fwd_config;
    TrackerConfig &side_config;
};

#endif  // FwdSideSensorTracker_H