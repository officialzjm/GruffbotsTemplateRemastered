#pragma once
#ifndef RAMSETTE_H
#define RAMSETTE_H

#include "api.h"
#include "utils.h"
#include "Eigen/Eigen"

class Ramsette {
    public:
        Ramsette(float b, float z);
        Eigen::Vector2f CalculateControl(const Pose& current, const Pose& target, const Eigen::Vector2f& target_velocity);
    private:
        float b; // Tunable parameter for convergence rate
        float z; // Damping coefficient
};

#endif  // RAMSETTE_H