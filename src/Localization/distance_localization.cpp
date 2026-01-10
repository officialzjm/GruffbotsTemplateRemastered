#include "distance_localization.h"

double DistanceLocalization::RaycastToWall(Point &ray_origin, float ray_angle, Wall &wall)
{
    double x0 = ray_origin.x;
    double y0 = ray_origin.y;
    double dx = cos(ray_angle);
    double dy = sin(ray_angle);

    double x1 = wall.start.x;
    double y1 = wall.start.y;
    double x2 = wall.end.x;
    double y2 = wall.end.y;

    double denominator = (x2 - x1) * dy - (y2 - y1) * dx;
    if(fabs(denominator) < 1e-6) {
        return -1.0; // Parallel lines
    }
    double t = ((x1 - x0) * (y2 - y1) - (y1 - y0) * (x2 - x1)) / denominator;
    double u = ((x1 - x0) * dy - (y1 - y0) * dx) / denominator;

    if(t >= 0 && u >= 0 && u <= 1) {
        double intersection_x = x0 + t * dx;
        double intersection_y = y0 + t * dy;
        double distance = sqrt(pow(intersection_x - x0, 2) + pow(intersection_y - y0, 2));
        return distance;
    } else {
        return -1.0; // No intersection
    }
}

double DistanceLocalization::GetExpectedDistanceToWall(Point robot_position, float sensor_heading, std::vector<Wall>* walls)
{
    double min_distance = std::numeric_limits<double>::max();
    for(auto& wall : *walls) {
        double distance = RaycastToWall(robot_position, sensor_heading, wall);
        if(distance >= 0 && distance < min_distance) {
            min_distance = distance;
        }
    }
    return (min_distance == std::numeric_limits<double>::max()) ? -1.0 : min_distance;
}

DistanceSensorWrapper::DistanceSensorWrapper(int port, SensorIndex label) : sensor(pros::Distance(port))
{
     if(port < 1 || port > 21) {
        throw std::invalid_argument("Port number must be between 1 and 21");
    }
}

double DistanceSensorWrapper::GetFilteredReading()
{
    int distance_mm = sensor.get_distance();
    int confidence = sensor.get_confidence();

    if(((double)rand() / RAND_MAX) < DistanceLocalizationConfig::DROPOUT_PROBABILITY) {
        pros::lcd::print(0, 0, "Sensor %d dropout simulated", index);
        return -1.0; // Simulate dropout
    }

    if(confidence < DistanceLocalizationConfig::CONFIDENCE_THRESHOLD) {
        pros::lcd::print(0, 0, "Sensor %d low confidence: %d", index, confidence);
        return -1.0; // Low confidence
    }

    if(distance_mm < DistanceLocalizationConfig::MIN_VALID_MM ||
        distance_mm > DistanceLocalizationConfig::MAX_VALID_MM ||
        confidence < DistanceLocalizationConfig::CONFIDENCE_THRESHOLD) {
        return -1.0; // Invalid reading
    }

    if(distance_mm < DistanceLocalizationConfig::MIN_VALID_MM || distance_mm > DistanceLocalizationConfig::MAX_VALID_MM) {
        pros::lcd::print(0, 0, "Sensor %d invalid reading: %d mm", index, distance_mm);
        return -1.0; // Invalid reading
    }

    double distance_inch = utils::convert_to_inches(static_cast<float>(distance_mm), utils::MILLIMETERS);
    return distance_inch;
}
