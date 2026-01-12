#pragma once
#include "Bezier.h"
#include <vector>
#include <string>
#include "Eigen/Eigen"
#include <fstream>
#include "json/json.h"

struct Event {
    double t;
    std::string name;
};

struct Path {
    double startSpeed;
    double endSpeed;
    std::vector<BezierSegment> segments;
    std::vector<Event> events;
};


class PathLoader {
public:
    // Load path JSON into segments and events
    static Path load(const std::string& file);
};
