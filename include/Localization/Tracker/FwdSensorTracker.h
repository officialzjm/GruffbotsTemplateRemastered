#pragma once
#ifndef FwdSensorTracker_H
#define FwdSensorTracker_H

#include "TrackerInterface.h"
#include "api.h"
#include "TrackerConfig.h"

class FwdSensorTracker : public TrackerInterface {
   public:
    FwdSensorTracker(TrackerConfig &fwd_config) : fwd_config(fwd_config) {};
    float getForwardDistance() override;
    float getSideDistance() override;
    float getFwdCenterDistance() override { return fwd_config.center_distance; };
    float getSideCenterDistance() override { return 0.0f; };

   private:
    TrackerConfig &fwd_config;
};

#endif  // FwdSensorTracker_H