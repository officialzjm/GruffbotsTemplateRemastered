#pragma once
#ifndef SPLINETRAJECTORYGENERATOR_H
#define SPLINETRAJECTORYGENERATOR_H

#include "api.h"
#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <stdexcept>
#include "utils.h"


//The coefficients for the spline equation we solve for
struct Coeffs {
    std::array<double,6> cx; // x coefficients
    std::array<double,6> cy; // y coefficients
};

class QuinticSpline {
    private:
        std::vector<Trajectory> waypoints;
        std::vector<Coeffs> segments;
        std::vector<double> dx, ddx;
        std::vector<double> dy, ddy;
        double dt = 0.005;
        double max_velocity;

        double EvalPoly(std::array<double,6> c, double t);
        double EvalPolyDerivative(const std::array<double,6>& c, double t);
        double EvalPolySecondDerivative(const std::array<double,6>& c, double t);
        std::array<double,6> computeCoeffs(double p0,double p1,
            double d0,double d1,
            double dd0,double dd1);
        void BuildCoefficients();
    public:
        QuinticSpline(const std::vector<Trajectory>& pts, double max_velocity = 10.0);
        //Trajectory GetTargetTrajectory(double t);
        Trajectory GetTrajectory(double t);
};


#endif //SPLINETRAJECTORYGENERATOR_H