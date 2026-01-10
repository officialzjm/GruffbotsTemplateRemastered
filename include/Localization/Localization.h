#pragma once
#ifndef LOCOMOTION_H
#define LOCOMOTION_H

#include "api.h"
#include "odom.h"

namespace LocalizationConfig {
    constexpr uint32_t ODOM_UPDATE_DELAY_MS = 20;
    constexpr uint32_t IMU_CALIBRATION_DELAY_MS = 10;
    constexpr double TRACK_WIDTH = 5.5;
}

class Localization {
    public:
        /// @brief Handles the odometry positioning system
        /// @param odom A pointer to the odometry class used
        Localization(Odom* odom);

        ///@brief Starts the odometry task
        void StartOdomTracking();
        ///@brief Stops the odometry task
        void StopOdomTracking();

        /// @brief Returns the robot's current position

        Pose GetGlobalPose() { return odom->GetGlobalPose(); }

        /// @brief Sets the robot's global position
        /// @param pose The desired pose to set

        void setGlobalPose(Pose pose) { odom->SetGlobalPose(pose); }
        void CalibrateIMU() { odom->CalibrateIMU(); }

    private:
        pros::Task* odom_task;             ///< Odometry tracking task pointer
        static void PositionTrackTask(void* chassis);
        void PositionTrack();
        Odom* odom;
};

#endif  // LOCOMOTION_H