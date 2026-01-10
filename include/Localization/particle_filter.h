#pragma once

#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include "utils.h"
#include "api.h"
#include "distance_localization.h"
#include "Localization.h"

namespace ParticleFilterConfig {
    constexpr float MOTION_NOISE_TRACKERS = 0.12f; // in inches
    constexpr float MOTION_NOISE_ENCODERS = 0.4f; // in inches
    constexpr uint32_t PF_UPDATE_DELAY_MS = 50;
}

enum TrackerType {
    ODOMETRY,
    ENCODERS
};

struct Particle {
    Pose pose;
    float weight;
    float log_weight;
};

class ParticleFilter {
    private:
        std::vector<Particle> particles;
        std::vector<Wall>* walls = nullptr;
        DistanceLocalization* d_loc;
        TrackerType tracker_type = ENCODERS;
        pros::Task* pf_task = nullptr;
        Localization* loc;
        Pose prev_pose = {0.0f, 0.0f, 0.0f};

        void InitializeParticles(int numParticles, float area_width, float area_height);
        void MotionUpdate(float delta_x, float delta_y, float delta_heading, float motion_noise);
        void SensorUpdate(std::vector<float> &sensor_readings, float sensor_noise);
        void ResampleParticles();

        static void ParticleFilterTask(void* pf_instance) {
            ParticleFilter* pf = static_cast<ParticleFilter*>(pf_instance);
            pf->ParticleFilterLoop();
        }

        void Update(float delta_x, float delta_y, float delta_heading, std::vector<float> &sensor_readings, float sensor_noise);
        void ParticleFilterLoop();

    public:
        ParticleFilter(DistanceLocalization* d_loc, Localization* loc, TrackerType tracker) : tracker_type(tracker)
        {
            this->d_loc = d_loc;
            this->loc = loc;
        }

        Particle EstimatePosition();
        void StartFilterTask(int numParticles, std::vector<Wall> *walls, float area_width, float area_height);
        void StopFilterTask();
    };

#endif //PARTICLE_FILTER_H