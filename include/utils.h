#ifndef UTILS_H
#define UTILS_H

/**
 * @brief Utility functions for VEX robotics
 *
 * Contains common mathematical and control functions used throughout the robot code.
 */
namespace utils {
    /**
     * @brief Apply exponential scaling to joystick input for better control
     *
     * @param value Raw joystick value (-127 to 127)
     * @return Scaled value with exponential response
     */
    float convert_joystick_exponential(float value);

    enum DistanceUnit {
        INCHES,
        CENTIMETERS,
        MILLIMETERS
    };

    float convert_to_inches(float value, DistanceUnit unit);

    /// @brief Get the sign of a value
    /// @param value the number to get the sign of
    /// @return 1, -1, or 0

    int sign(float value);

    /**
     * @brief Convert percentage to motor voltage
     *
     * @param value Percentage value (-100 to 100)
     * @return Motor voltage (-12000 to 12000)
     */
    float convert_percent_to_voltage(float value);

    /**
     * @brief Clamp a value between min and max bounds
     *
     * @param value Input value
     * @param min Minimum bound
     * @param max Maximum bound
     * @return Clamped value
     */
    float clamp(float value, float min, float max);

    /**
     * @brief Convert degrees to radians
     *
     * @param degrees Angle in degrees
     * @return Angle in radians
     */
    float toRadians(float degrees);

    /**
     * @brief Convert radians to degrees
     *
     * @param radians Angle in radians
     * @return Angle in degrees
     */
    float toDegrees(float radians);
}

/**
 * @brief 2D point structure
 */
struct Point {
    float x;  ///< X coordinate
    float y;  ///< Y coordinate
};

/**
 * @brief Robot pose structure
 */
struct Pose {
    float x;       ///< X position in inches
    float y;       ///< Y position in inches
    float heading; ///< Heading in degrees

    bool operator==(const Pose& other) const {
        return (x == other.x) && (y == other.y) && (heading == other.heading);
    }
};

struct State {
    Pose position;     ///< Position in inches
    float velocity;     ///< Velocity in inches per second
    float acceleration; ///< Acceleration in inches per second squared
};

struct Trajectory {
    Pose position;     ///< Position in inches
    float linearVelocity;     ///< Velocity in inches per second
    float angularVelocity;
    float time;

    Trajectory(Pose position, float linearVelocity, float angularVelocity, float time):
    position(position), linearVelocity(linearVelocity), angularVelocity(angularVelocity),
    time(time)
    {}
};


/**
 * @brief Polar coordinate structure
 */
struct Polar {
    float r;     ///< Radius/distance
    float theta; ///< Angle in radians
};

#endif  // UTILS_H
