#pragma once
#include "MotionProfile.h"
//#include "TrajectorySampler.h"
//#include "VelocityProfile.h"
#include <vector>

class GeneratedProfile : public MotionProfile {
    std::vector<MotionCommand> table;
    double totalTime;

    void build(const std::vector<MotionCommand>& samples, const std::vector<double>& v);

public:
    GeneratedProfile(const std::vector<MotionCommand>& samples, const std::vector<double>& v);

    std::optional<MotionCommand> get(double time) override;
    double duration() const override { return totalTime; }
};
