#pragma once
#include "Eigen/Eigen"
#include <cmath>

struct BezierSegment {
    Eigen::Vector2d p0, p1, p2, p3;  // Control points (x,y)
    double maxVel, maxAccel;
    bool reversed = false;

    BezierSegment() = default;
    BezierSegment(Eigen::Vector2d p0, Eigen::Vector2d p1, Eigen::Vector2d p2, Eigen::Vector2d p3, 
                  double maxVel, double maxAccel, bool reversed = false)
        : p0(p0), p1(p1), p2(p2), p3(p3), maxVel(maxVel), maxAccel(maxAccel), reversed(reversed) {}

    // Evaluate position at t [0,1]
    Eigen::Vector3d pose(double t) const {
        double u = 1.0 - t;
        double tt = t * t;
        double uu = u * u;
        double uuu = uu * u;
        double ttt = tt * t;

        double x = uuu * p0.x() + 3 * uu * t * p1.x() + 3 * u * tt * p2.x() + ttt * p3.x();
        double y = uuu * p0.y() + 3 * uu * t * p1.y() + 3 * u * tt * p2.y() + ttt * p3.y();

        // Calculate tangent for heading (derivative)
        double dx = 3 * uu * (p1.x() - p0.x()) + 6 * u * t * (p2.x() - p1.x()) + 3 * tt * (p3.x() - p2.x());
        double dy = 3 * uu * (p1.y() - p0.y()) + 6 * u * t * (p2.y() - p1.y()) + 3 * tt * (p3.y() - p2.y());
        double theta = std::atan2(dy, dx);

        return Eigen::Vector3d(x, y, theta);
    }

    // First derivative (velocity vector) for curvature
    Eigen::Vector2d derivative(double t) const {
        double u = 1.0 - t;
        double tt = t * t;
        double uu = u * u;

        double dx = 3 * uu * (p1.x() - p0.x()) + 6 * u * t * (p2.x() - p1.x()) + 3 * tt * (p3.x() - p2.x());
        double dy = 3 * uu * (p1.y() - p0.y()) + 6 * u * t * (p2.y() - p1.y()) + 3 * tt * (p3.y() - p2.y());
        return Eigen::Vector2d(dx, dy);
    }

    // Second derivative for curvature calculation
    Eigen::Vector2d secondDerivative(double t) const {
        double u = 1.0 - t;
        double tt = t * t;

        double dx = 6 * u * (p2.x() - 2 * p1.x() + p0.x()) + 6 * tt * (p3.x() - 2 * p2.x() + p1.x());
        double dy = 6 * u * (p2.y() - 2 * p1.y() + p0.y()) + 6 * tt * (p3.y() - 2 * p2.y() + p1.y());
        return Eigen::Vector2d(dx, dy);
    }

    // Curvature κ = |x' y'' - y' x''| / (x'^2 + y'^2)^(3/2)
    double curvature(double t) const {
        auto v1 = derivative(t);
        auto v2 = secondDerivative(t);
        double num = std::abs(v1.x() * v2.y() - v1.y() * v2.x());
        double den = std::pow(v1.squaredNorm(), 1.5);
        return den > 1e-6 ? num / den : 0.0;
    }

    // Approximate arc length by sampling
    double arcLength() const {
        return arcLengthAt(1.0);
    }

    double arcLengthAt(double t) const {
        const int N = 50;
        double length = 0.0;
        double prevX = p0.x(), prevY = p0.y();
        
        for (int i = 1; i <= N; ++i) {
            double u = t * i / N;
            Eigen::Vector3d pos = pose(u);
            length += std::hypot(pos.x() - prevX, pos.y() - prevY);
            prevX = pos.x(); prevY = pos.y();
        }
        return length;
    }
};
