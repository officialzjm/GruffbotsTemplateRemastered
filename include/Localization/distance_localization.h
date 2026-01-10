#pragma once 

#ifndef DISTANCE_LOCALIZATION_H
#define DISTANCE_LOCALIZATION_H

#include "api.h"
#include "utils.h"

namespace DistanceLocalizationConfig {
    constexpr float SENSOR_NOISE = 1.0f; // in inches
    constexpr int MIN_VALID_MM = 20;       // Sensor unreliable below 20 mm
    constexpr int MAX_VALID_MM = 2000;     // Sensor max reliable range
    constexpr int CONFIDENCE_THRESHOLD = 30; // Confidence out of 63
    constexpr double DROPOUT_PROBABILITY = 0.05; // 5% chance to simulate dropout

}

enum SensorIndex {
    FRONT = 0,
    BACK = 1,
    LEFT = 2,
    RIGHT = 3
};

struct Wall {
    Point start;
    Point end;
    std::string label;
};

class DistanceSensorWrapper {
    private:
        pros::Distance sensor;
        SensorIndex index;
    public:
        DistanceSensorWrapper(int port, SensorIndex label);
        double GetFilteredReading();
};

class DistanceLocalization {
    private:
        std::vector<DistanceSensorWrapper>& sensors;

        double RaycastToWall(Point& ray_origin, float ray_angle, Wall& wall);
    public:
        DistanceLocalization(std::vector<DistanceSensorWrapper>& sensors) :
            sensors(sensors) {}     

        double GetExpectedDistanceToWall(Point robot_position, float sensor_heading, std::vector<Wall>* wall);
    
        std::vector<float> GetSensorReadings() {
            return {
                utils::convert_to_inches(static_cast<float>(sensors[FRONT].GetFilteredReading()), utils::MILLIMETERS),
                utils::convert_to_inches(static_cast<float>(sensors[BACK].GetFilteredReading()), utils::MILLIMETERS),
                utils::convert_to_inches(static_cast<float>(sensors[LEFT].GetFilteredReading()), utils::MILLIMETERS),
                utils::convert_to_inches(static_cast<float>(sensors[RIGHT].GetFilteredReading()), utils::MILLIMETERS)
            };
        }
};

#endif //DISTANCE_LOCALIZATION_H