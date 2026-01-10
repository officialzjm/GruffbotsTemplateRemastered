#include "odom.h"
#include "utils.h"
#include <cmath>
#include "Localization.h"

void Odom::Initialize(float start_x, float start_y, float start_heading)
{
    global_pose.x = start_x;
    global_pose.y = start_y;
    global_pose.heading = start_heading;
    prev_forward_distance = 0;
    prev_side_distance = 0;
    prev_heading_rads = start_heading;
}

void Odom::Set_Physical_Distances(float side_distance, float forward_distance)
{
    side_tracker_center_distance = side_distance;
    forward_tracker_center_distance = forward_distance;
}

void Odom::Update_Coordinates()
{
    float current_heading_rads = utils::toRadians(imu.get_heading()); //+ global_pose.heading;
    float forward_distance = tracker.getForwardDistance();
    float side_distance = tracker.getSideDistance();
    float forward_delta = forward_distance - this->prev_forward_distance;
    float side_delta = side_distance - this->prev_side_distance;
    this->prev_forward_distance = forward_distance;
    this->prev_side_distance = side_distance;

    float heading_delta_rad = current_heading_rads - prev_heading_rads;
    this->prev_heading_rads = current_heading_rads;

    float local_X = 0;
    float local_Y = 0;

    if(heading_delta_rad == 0) {
        local_Y = side_delta;
        local_X = forward_delta;
    } else {
        local_Y = (2*sin(heading_delta_rad/2))*((side_delta/heading_delta_rad)+side_tracker_center_distance); 
        local_X = (2*sin(heading_delta_rad/2))*((forward_delta/heading_delta_rad)+forward_tracker_center_distance);
    }

    Polar local_polar = {0,0};

    if(local_X == 0 && local_Y == 0) {
        local_polar.r = 0;
        local_polar.theta = 0;
    } else {
        local_polar.r = sqrt((local_X*local_X)+(local_Y*local_Y));
        local_polar.theta = atan2(local_Y, local_X);
    }

    float global_polar_angle = local_polar.theta - prev_heading_rads - (heading_delta_rad/2);

    Point global_delta = {0,0};
    global_delta.x = local_polar.r * cos(global_polar_angle);
    global_delta.y = local_polar.r * sin(global_polar_angle);

    global_pose.x += global_delta.x;
    global_pose.y += global_delta.y;
    while(current_heading_rads > M_PI) current_heading_rads -= M_PI*2;
    while(current_heading_rads < -M_PI) current_heading_rads += M_PI*2;
    global_pose.heading = -current_heading_rads;
}

void Odom::CalibrateIMU()
{
    if (!imu.is_calibrating()) {
        imu.reset();
        while (imu.is_calibrating()) {
            pros::delay(LocalizationConfig::IMU_CALIBRATION_DELAY_MS);
        }
    }
}