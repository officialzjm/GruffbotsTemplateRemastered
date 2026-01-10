#include "particle_filter.h"

void ParticleFilter::InitializeParticles(int numParticles, float area_width, float area_height)
{
    particles.clear();
    particles.resize(numParticles);

    for(int i = 0; i < numParticles; i++) {
        Particle p;
        p.pose.x = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/area_width));
        p.pose.y = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/area_height));
        p.pose.heading = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/360.0f));
        p.weight = 1.0f / numParticles;
        particles[i] = p;
    }
}

void ParticleFilter::MotionUpdate(float delta_x, float delta_y, float delta_heading, float motion_noise)
{
    for(auto& particle : particles) {
        float noisy_delta_x = delta_x + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/(2*motion_noise))) - motion_noise;
        float noisy_delta_y = delta_y + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/(2*motion_noise))) - motion_noise;
        float noisy_delta_heading = delta_heading + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/(2*motion_noise))) - motion_noise;

        particle.pose.x += noisy_delta_x;
        particle.pose.y += noisy_delta_y;
        particle.pose.heading += noisy_delta_heading;

        while(particle.pose.heading >= 2*M_PI) particle.pose.heading -= 2*M_PI;
        while(particle.pose.heading < 0.0f) particle.pose.heading += 2*M_PI;
    }
}

void ParticleFilter::SensorUpdate(std::vector<float> &sensor_readings, float sensor_noise)
{
    for(auto& p : particles) {
        double expected_front = d_loc->GetExpectedDistanceToWall({p.pose.x, p.pose.y}, utils::toRadians(p.pose.heading), walls);
        double expected_back = d_loc->GetExpectedDistanceToWall({p.pose.x, p.pose.y}, utils::toRadians(p.pose.heading + 180.0f), walls);
        double expected_left = d_loc->GetExpectedDistanceToWall({p.pose.x, p.pose.y}, utils::toRadians(p.pose.heading + 90.0f), walls);
        double expected_right = d_loc->GetExpectedDistanceToWall({p.pose.x, p.pose.y}, utils::toRadians(p.pose.heading - 90.0f), walls);

        double sigma = 1;
        double weight = 1;
        double log_weight = 0;

        if(sensor_readings[FRONT] > 0) {
            double error = sensor_readings[FRONT] - expected_front;
            log_weight += - (error * error) / (2 * sensor_noise * sensor_noise);
        }

        if(sensor_readings[BACK] > 0) {
            double error = sensor_readings[BACK] - expected_back;
            log_weight += - (error * error) / (2 * sensor_noise * sensor_noise);
        }

        if(sensor_readings[LEFT] > 0) {
            double error = sensor_readings[LEFT] - expected_left;
            log_weight += - (error * error) / (2 * sensor_noise * sensor_noise);
            }

        if(sensor_readings[RIGHT] > 0) {
            double error = sensor_readings[RIGHT] - expected_right;
            log_weight += - (error * error) / (2 * sensor_noise * sensor_noise);
        }

        p.log_weight = log_weight;
    }

      double max_log_weight = -INFINITY;

      for(const auto& p : particles) {
          if(p.log_weight > max_log_weight) max_log_weight = p.log_weight;
      }

    double total_weight = 0.0;
    for(auto& p : particles) {
        p.weight = exp(p.log_weight - max_log_weight);
        total_weight += p.weight;
    }

    for(auto& p : particles) {
        p.weight /= total_weight;
    }
}

void ParticleFilter::ResampleParticles()
{
    std::vector<Particle> new_particles;
    int N = particles.size();

    double step = 1.0 / N;
    double r = (static_cast<double>(rand()) / RAND_MAX) * step;
    double c = particles[0].weight;
    int i = 0;

    for (int m = 0; m < N; m++) {
        double U = r + m * step;
        while (U > c && i < N - 1) {
            i++;
            c += particles[i].weight;
        }
        new_particles.push_back(particles[i]);
    }
   
    particles = new_particles;

    for(auto& p : particles) {
        p.weight = 1.0f / N;
        p.log_weight = 0;
    }
}

void ParticleFilter::ParticleFilterLoop()
{
    while(this->pf_task != nullptr && this->pf_task->get_state() != pros::E_TASK_STATE_DELETED) {
        Pose current_pose = loc->GetGlobalPose();
        float delta_x = current_pose.x - prev_pose.x;
        float delta_y = current_pose.y - prev_pose.y;
        float delta_heading = current_pose.heading - prev_pose.heading;

        std::vector<float> sensor_readings = d_loc->GetSensorReadings();

        this->Update(delta_x, delta_y, delta_heading, sensor_readings, DistanceLocalizationConfig::SENSOR_NOISE);
        prev_pose = current_pose;
        
        pros::delay(ParticleFilterConfig::PF_UPDATE_DELAY_MS);
    }
}

Particle ParticleFilter::EstimatePosition()
{
    Pose pose = {0.0f, 0.0f, 0.0f};
    for(const auto& p : particles) {
        pose.x += p.pose.x * p.weight;
        pose.y += p.pose.y * p.weight;
        pose.heading += p.pose.heading * p.weight;
    }
    return {pose, 1.0f};
}

void ParticleFilter::Update(float delta_x, float delta_y, float delta_heading, std::vector<float> &sensor_readings, float sensor_noise)
{
    float motion_noise = (tracker_type == ENCODERS) ? ParticleFilterConfig::MOTION_NOISE_ENCODERS : ParticleFilterConfig::MOTION_NOISE_TRACKERS;
    MotionUpdate(delta_x, delta_y, delta_heading, motion_noise);
    SensorUpdate(sensor_readings, sensor_noise);
    ResampleParticles();
}


void ParticleFilter::StartFilterTask(int numParticles, std::vector<Wall>* walls, float area_width, float area_height)
{
    InitializeParticles(numParticles, area_width, area_height);
    this->walls = walls;

    if (this->pf_task == nullptr) {
        this->pf_task = new pros::Task(ParticleFilterTask, this, "Particle Filter Task");
    }
}

void ParticleFilter::StopFilterTask()
{
    if (this->pf_task != nullptr) {
        this->pf_task->remove();
        delete this->pf_task;
        this->pf_task = nullptr;
    }
}
