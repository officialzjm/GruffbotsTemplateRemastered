#pragma once
#ifndef ODOM_H
#define ODOM_H

#include "utils.h"
#include "Localization/Tracker/TrackerInterface.h"

/**
 * @brief Odometry calculation class
 *
 * Tracks robot position and orientation using encoder and IMU data.
 * Implements dead reckoning odometry for VEX robots.
 */
class Odom {
private:
    Pose global_pose;                       ///< Current global pose (x, y, heading)
    float prev_forward_distance;                 ///< Last recorded forward distance
    float prev_side_distance;                    ///< Last recorded side distance
    float prev_heading_rads;                  ///< Last recorded heading
    float side_tracker_center_distance;     ///< Distance from robot center to side tracker
    float forward_tracker_center_distance;  ///< Distance from robot center to forward tracker
    TrackerInterface &tracker;          ///< Tracker interface for encoder readings
    pros::IMU &imu;                          ///< IMU reference for heading

public:
    
    Odom(TrackerInterface &tracker, pros::IMU &imu) : prev_forward_distance(tracker.getForwardDistance()),
        prev_side_distance(tracker.getSideDistance()), prev_heading_rads(0.0f), tracker(tracker), imu(imu) {
        side_tracker_center_distance = tracker.getSideCenterDistance();
        forward_tracker_center_distance = tracker.getFwdCenterDistance();
        global_pose = {0.0f, 0.0f, 0.0f};
    }

    void Initialize(float start_x, float start_y, float start_heading);

    void Set_Physical_Distances(float side_distance, float forward_distance);

    void Update_Coordinates();

    Pose GetGlobalPose() const { return global_pose; }
    void SetGlobalPose(const Pose &new_pose) { global_pose = new_pose; }
    void CalibrateIMU();

    /**
     * @brief Reset odometry to origin
     */
    void Reset() {
        global_pose = {0.0f, 0.0f, 0.0f};
        prev_forward_distance = 0.0f;
        prev_side_distance = 0.0f;
        prev_heading_rads = 0.0f;
    }
};

#endif  // ODOM_H
