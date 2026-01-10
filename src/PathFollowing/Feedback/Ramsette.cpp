#include "Ramsette.h"
#include "utils.h"

Ramsette::Ramsette(float b, float z):
    b(b), z(z)
{
}


Eigen::Vector2f Ramsette::CalculateControl(const Pose &current, const Pose &target, const Eigen::Vector2f &target_velocity)
{    
    Eigen::Vector3f error;
    error = Eigen::Vector3f(target.x - current.x,
                            target.y - current.y,
                            target.heading - current.heading);

    // Normalize heading error to [-pi, pi]
    while (error(2) > M_PI) error(2) -= 2 * M_PI;
    while (error(2) < -M_PI) error(2) += 2 * M_PI;
    // double new_heading_error = 0;
    // if(error(2) < 0) new_heading_error = -1/3 * pow(error(2),2);
    // else new_heading_error = 1/3 * pow(error(2),2); 

    // error(2) = new_heading_error;

    Eigen::Matrix3f transform_to_local_error = Eigen::Matrix3f::Identity();
    transform_to_local_error(0, 0) = cos(current.heading);
    transform_to_local_error(0, 1) = sin(current.heading);
    transform_to_local_error(1, 0) = -sin(current.heading);
    transform_to_local_error(1, 1) = cos(current.heading);

    Eigen::Vector3f local_error = transform_to_local_error * error;

    while (local_error(2) > M_PI) local_error(2) -= 2 * M_PI;
    while (local_error(2) < -M_PI) local_error(2) += 2 * M_PI;

    float k = 2 * z * sqrt( (b * pow(target_velocity(0),2))  +   pow(target_velocity(1),2)  );
    float v = target_velocity(0) * cos(local_error(2)) + k * local_error(0);
    float sinc_theta = (fabs(local_error(2)) < 1e-5f) ? 1.0f : sin(local_error(2)) / local_error(2);
    float omega = target_velocity(1) + k * local_error(2) + (b * target_velocity(0) * sinc_theta * local_error(1));
   // pros::lcd::print(6, "error: %.2f, %.2f, %.2f %.2f", error(0), error(1), error(2), current.heading);

        
    // omega = utils::clamp(omega, -1, 1);
    return Eigen::Vector2f(v, omega);
    
}

