#include "api.h"
#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <stdexcept>
#include "utils.h"
#include "SplineTrajectoryGenerator.h"


double QuinticSpline::EvalPoly(const std::array<double,6> c, double t) {
    double res = 0, tp = 1;
    for(int i=0;i<6;i++){ res += c[i]*tp; tp*=t; }
    return res;
}

double QuinticSpline::EvalPolyDerivative(const std::array<double,6>& c, double t) {
    double res = 0, power = 1;
    for (int i=1; i<6; i++) {
        res += i * c[i] * power;
        power *= t;
    }
    return res;
}
double QuinticSpline::EvalPolySecondDerivative(const std::array<double, 6> &c, double t)
{
    double res = 0.0;
    double power = 1.0;
    for (int i=2; i<6; i++) {
        res += i * (i - 1) * c[i] * power;
        power *= t;
    }
    return res;
}
std::array<double, 6> QuinticSpline::computeCoeffs(double p0, double p1,
                                                   double d0, double d1,
                                                   double dd0, double dd1)
{
    std::array<double,6> c;
    c[0] = p0;
    c[1] = d0;
    c[2] = 0.5*dd0;
    c[3] = -10*p0 -6*d0 -1.5*dd0 +10*p1 -4*d1 +0.5*dd1;
    c[4] = 15*p0 +8*d0 +1.5*dd0 -15*p1 +7*d1 - dd1;
    c[5] = -6*p0 -3*d0 -0.5*dd0 +6*p1 -3*d1 +0.5*dd1;
    return c;
}

void QuinticSpline::BuildCoefficients() {
    int N = waypoints.size()-1;
    segments.resize(N);
    for(int k=0;k<N;k++){
        segments[k].cx = computeCoeffs(
            waypoints[k].position.x, waypoints[k+1].position.x,
            dx[k], dx[k+1], ddx[k], ddx[k+1]);
        segments[k].cy = computeCoeffs(
            waypoints[k].position.y, waypoints[k+1].position.y,
            dy[k], dy[k+1], ddy[k], ddy[k+1]);
    }
}



QuinticSpline::QuinticSpline(const std::vector<Trajectory>& pts, double max_velocity)
{
    waypoints = pts;
    int N = pts.size()-1; 
    dx.resize(N+1);
    ddx.resize(N+1, 0); // curvature second derivatives - zero or estimated
    dy.resize(N+1);
    ddy.resize(N+1, 0);

    this->max_velocity = max_velocity;

    for (int i = 0; i <= N; ++i) {
        double scale = 0.5; //tune
        double dist = 0.0;
        if (i < N) {
            dist += std::hypot(waypoints[i+1].position.x - waypoints[i].position.x, waypoints[i+1].position.y - waypoints[i].position.y);
        }
        if (i > 0) {
            dist += std::hypot(waypoints[i].position.x - waypoints[i-1].position.x, waypoints[i].position.y - waypoints[i-1].position.y);
        }
        dx[i] = std::cos(waypoints[i].position.heading) * dist * scale;
        dy[i] = std::sin(waypoints[i].position.heading) * dist * scale;
    }
    
    for (int i = 0; i <= N; ++i) {
        double dx = waypoints[i+1].position.x - waypoints[i].position.x;
        double dy = waypoints[i+1].position.y - waypoints[i].position.y;
        double dist = std::sqrt(dx * dx + dy * dy);
        double dt = waypoints[i+1].time - waypoints[i].time;

        double v = (dt > 1e-6) ? (dist / dt) : max_velocity;

        if (v > max_velocity) v = max_velocity;

        waypoints[i].linearVelocity = v;
    }
    /*
    // Convert heading angles to tangent vectors (dx, dy)
    for (int i=0; i<=N; i++) {
        dx[i] = std::cos(waypoints[i].position.heading) * waypoints[i].linearVelocity;
        dy[i] = std::sin(waypoints[i].position.heading) * waypoints[i].linearVelocity;
    }
    */

    BuildCoefficients();
}

Trajectory QuinticSpline::GetTrajectory(double t) {
    
    int N = waypoints.size()-1;
    if (t <= waypoints[0].time) t = waypoints[0].time;
    if (t >= waypoints[N].time) t = waypoints[N].time;

    int k = 0;
    for (int i = 0; i < N; ++i) {
        if (t >= waypoints[i].time && t <= waypoints[i+1].time) {
            k = i;
            break;
        }
    }

    
    double t0 = waypoints[k].time, t1 = waypoints[k+1].time;
    double tau = (t - t0) / (t1 - t0); 
    
    
    
    //position
    const auto seg = segments[k];
    double xt = EvalPoly(seg.cx, tau);
    double yt = EvalPoly(seg.cy, tau);
    
    //heading
    
    double dx_dtau = EvalPolyDerivative(seg.cx, tau);
    double dy_dtau = EvalPolyDerivative(seg.cy, tau);

    double scale = 1.0 / (t1 - t0); 
    double dx_dt = dx_dtau * scale;
    double dy_dt = dy_dtau * scale;

    double heading = std::atan2(dy_dt, dx_dt);
    double linear_velocity = std::sqrt(dx_dt * dx_dt + dy_dt * dy_dt);

    double ddx_dtau = EvalPolySecondDerivative(seg.cx, tau);
    double ddy_dtau = EvalPolySecondDerivative(seg.cy, tau);
    double ddx_dt = ddx_dtau * scale * scale;
    double ddy_dt = ddy_dtau * scale * scale;

    double angular_velocity = (dx_dt * ddy_dt - dy_dt * ddx_dt) / (linear_velocity * linear_velocity);

    //angular_velocity = utils::clamp(angular_velocity, -0.8, 0.8);
    Trajectory traj = Trajectory({xt, yt, heading}, linear_velocity, angular_velocity, t);
    return traj;
}


// Trajectory QuinticSpline::GetTargetTrajectory(double t)
// {
//     Pose current = getPose(t);
//     Pose current_dt = getPose(t+dt);

//     double dx = current_dt.x - current.x;
//     double dy = current_dt.y - current.y;
//     double dh = current_dt.heading - current.heading;

//     double desired_linear_velocity = sqrt(pow(dx/dt, 2) + pow(dy/dt, 2));
//     double desired_angular_velocity = dh/dt;

//     double  dx_dtau = 


//     return Trajectory(current, desired_linear_velocity, desired_angular_velocity, t);
// }