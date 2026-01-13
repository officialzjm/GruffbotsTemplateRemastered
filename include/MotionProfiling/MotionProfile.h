#pragma once
#include "PathInterpreter.h"
#include "Eigen/Eigen"
#include <vector>
#include <optional>

struct MotionCommand {
    Eigen::Vector3d desiredPose;
    double desiredVel;
    double desiredAngVel;
    double desiredT;
};

class MotionProfile {
private:
    std::vector<BezierSegment> segments;
    std::vector<double> times, pathTs, velocities;
    double duration = 0.0;
    
    static double lerp(const std::vector<double>& xs, const std::vector<double>& ys, double x) {
        if (x <= xs[0]) return ys[0];
        if (x >= xs.back()) return ys.back();
        for (size_t i = 1; i < xs.size(); ++i) {
            if (xs[i] >= x) {
                double t = (x - xs[i-1]) / (xs[i] - xs[i-1]);
                return (1-t)*ys[i-1] + t*ys[i];
            }
        }
        return ys.back();
    }
    
    double speedLimit(double kappa) const {
        if (std::abs(kappa) < 1e-6) return 2.0;
        return 2.0 / (1.0 + std::abs(kappa) * 0.45 * 0.5);
    }
    
public:
    MotionProfile(const Path& path) {
        segments = path.segments;
        if (segments.empty()) return;
                
        std::vector<double> distances{0.0};
        std::vector<double> pathTsTemp;
        std::vector<double> vels;
        double totalDist = 0.0;
        
        for (size_t i = 0; i < segments.size(); ++i) {
            double len = segments[i].totalArcLength();
            int n = std::max(8, (int)(len * 20));
            for (int j = 1; j <= n; ++j) {
                double t = j/(double)n;
                pathTsTemp.push_back(i + t); 
                distances.push_back(totalDist + segments[i].arcLengthAtT(t));
                double kappa = segments[i].curvature(t);
                vels.push_back(std::min(segments[i].maxVel, speedLimit(kappa)));
            }
            totalDist += len;
        }
        
        if (!vels.empty()) {
            vels.back() = std::min(path.endSpeed, vels.back());
        }
        
        times.reserve(vels.size());
        pathTs.reserve(vels.size());
        velocities.reserve(vels.size());
        
        times.push_back(0.0)
        pathTs.push_back(0.0);
        velocities.push_back(path.startSpeed);
        
        double t = 0.0;
        for (size_t i = 1; i < vels.size(); ++i) {  // Safe now!
            double ds = distances[i] - distances[i-1];
            double vavg = (vels[i-1] + vels[i]) * 0.5;
            t += ds / std::max(vavg, 0.001);
            times.push_back(t);
            pathTs.push_back(pathTsTemp[i]);
            velocities.push_back(vels[i]);
        }
        duration = t;
    }

    
    std::optional<MotionCommand> get(double t) {
        if (times.empty() || t > duration) return std::nullopt;
        t = std::min(t, duration);
        
        double pathT = lerp(times, pathTs, t);
        double v = lerp(times, velocities, t);
        size_t i = std::min<size_t>(std::floor(pathT), segments.size() - 1);
        double localT = pathT - i;
        
        bool reversed = segments[i].reversed; 

        Eigen::Vector3d pose = segments[i].pose(localT);
        if (reversed) pose.z() += M_PI;
        
        double kappa = segments[i].curvature(localT);
        double omega = v * kappa;
        double vSign = reversed ? -1 : 1;
        
        return MotionCommand{pose, vSign * v, omega, pathT};
    }
    
    double getDuration() const { return duration; }
};
