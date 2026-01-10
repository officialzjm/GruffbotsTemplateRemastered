#include "utils.h"
#include <cmath>

float utils::convert_joystick_exponential(float value)
{
    if(value < 0) {
		value *= -1;
		value = (1.2*pow(1.043, value)) + (0.2 * value) - 1.2;
		value *= -1;
	}
	else {
		value = (1.2*pow(1.043, value)) + (0.2 * value) - 1.2;
	}
	return value;
}

float utils::convert_to_inches(float value, DistanceUnit unit)
{
	if (unit == CENTIMETERS) {
		return value / 2.54f;
	}
	else if (unit == MILLIMETERS) {
		return value / 25.4f;
	}
	return value;
}

int utils::sign(float value)
{
	if (value > 0) return 1;
	if (value < 0) return -1;
    return 0;
}

float utils::convert_percent_to_voltage(float value) {
    value *= 120;
	return value;
}

float utils::clamp(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

float utils::toRadians(float degrees)
{
    return degrees * (M_PI / 180.0f);
}

float utils::toDegrees(float radians)
{
    return radians * (180.0f / M_PI);
}
