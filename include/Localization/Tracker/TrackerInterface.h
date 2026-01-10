#pragma once
#ifndef TRACKERINTERFACE_H_
#define TRACKERINTERFACE_H_

#include "api.h"

class TrackerInterface {
   public:
    virtual float getForwardDistance() = 0;
    virtual float getSideDistance() = 0;
    virtual float getFwdCenterDistance() = 0;
    virtual float getSideCenterDistance() = 0;
};

#endif  // TRACKERINTERFACE_H_