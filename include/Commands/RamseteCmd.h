#pragma once

#include "Commands/Command.h"
#include "ConfiguredMotorGroup.h"
#include "MotionProfiling/MotionProfile.h"
#include "Localization/Localization.h"
#include "Eigen/Eigen"
#include "PathFollowing/Feedback/PID.h"

/**
 * Class to follow motion profiled paths
 */
class Ramsete : public Command {
private:
    ConfiguredMotorGroup *LeftDrive;
    ConfiguredMotorGroup *RightDrive;
    Localization *loc;
    MotionProfile *motionProfile;

    double zeta = 0.7;
    double beta = 2.0;
    double startTime = 0.0;

    PID left_PID  = PID(0.3, 0.02, 0.005);
	PID right_PID = PID(0.3, 0.02, 0.005);

    double prev_time = 0.0;

public:
    Ramsete(ConfiguredMotorGroup *LeftDrive, ConfiguredMotorGroup *RightDrive, Localization *loc, MotionProfile *motion_profile, float zeta, float beta) :
        LeftDrive(LeftDrive), RightDrive(RightDrive), loc(loc), motionProfile(motion_profile), zeta(zeta), beta(beta), startTime(0.0) {}
    

    void initialize() override { startTime = pros::millis() / 1000.0; }

    void execute() override {
        double current_time = pros::millis() / 1000.0;
        if (const auto command = motionProfile->get(current_time - startTime); command.has_value()) {
            float PID_settle_error = 700;
            
            double dt = current_time - prev_time;  // Calculate timestep
            prev_time = current_time;
            
            Pose currentPose = loc->GetGlobalPose();
            Eigen::Vector3f desiredPose = command->desiredPose.cast<float>();

            Eigen::Vector3f error;
            error = Eigen::Vector3f(desiredPose(0) - currentPose.x,
                                    desiredPose(1) - currentPose.y,
                                    desiredPose(2) - currentPose.heading);

            while (error(2) > M_PI) error(2) -= 2 * M_PI;
            while (error(2) < -M_PI) error(2) += 2 * M_PI;

            Eigen::Matrix3f transform_to_local_error = Eigen::Matrix3f::Identity();
            transform_to_local_error(0, 0) = cos(currentPose.heading);
            transform_to_local_error(0, 1) = sin(currentPose.heading);
            transform_to_local_error(1, 0) = -sin(currentPose.heading);
            transform_to_local_error(1, 1) = cos(currentPose.heading);

            Eigen::Vector3f local_error = transform_to_local_error * error;

            while (local_error(2) > M_PI) local_error(2) -= 2 * M_PI;
            while (local_error(2) < -M_PI) local_error(2) += 2 * M_PI;

            float k = 2 * zeta * sqrt( (beta * pow(command->desiredVel,2))  +   pow(command->desiredAngVel,2)  );
            float v = command->desiredVel * cos(local_error(2)) + k * local_error(0);
            float sinc_theta = (fabs(local_error(2)) < 1e-5f) ? 1.0f : sin(local_error(2)) / local_error(2);
            float omega = command->desiredAngVel + k * local_error(2) + (beta * command->desiredVel * sinc_theta * local_error(1));
            
            
            double targetLeftVel = v - omega * (LocalizationConfig::TRACK_WIDTH / 2.0);
            double targetRightVel = v + omega * (LocalizationConfig::TRACK_WIDTH / 2.0);

            MotorConfig left_config = LeftDrive->getConfig();
            MotorConfig right_config = RightDrive->getConfig();
            const double wheelCirc = M_PI * left_config.wheel_diameter;

            targetLeftVel = (targetLeftVel / wheelCirc) * 60.0 * left_config.gear_ratio;
            targetRightVel = (targetRightVel / wheelCirc) * 60.0 * right_config.gear_ratio;

            double actualLeftVel = LeftDrive->getMotorGroup().get_actual_velocity();
            double actualRightVel = RightDrive->getMotorGroup().get_actual_velocity();

            left_PID.UpdateTarget(targetLeftVel, PID_settle_error);
            right_PID.UpdateTarget(targetRightVel, PID_settle_error);
            double leftVoltage = left_PID.UpdateValues(actualLeftVel);
            double rightVoltage = right_PID.UpdateValues(actualRightVel);

        
            LeftDrive->getMotorGroup().move_voltage(leftVoltage);
            RightDrive->getMotorGroup().move_voltage(rightVoltage); 
        }
    }

    void end(bool interrupted) override { std::cout << "DONE" << std::endl; }

    bool isFinished() override { return motionProfile->getDuration() < pros::millis() / 1000.0 - startTime; }

    //std::vector<Subsystem *> getRequirements() override { return {drivetrain}; }
};