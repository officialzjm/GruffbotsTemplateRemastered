#pragma once
#include "Bezier.h"
#include <vector>
#include <string>
#include "Eigen/Eigen"
#include <fstream>
#include "json/json.h"
#include <stdint>
#include <cstddef>

struct asset {
	const uint8_t* data;
	size_t size;
};

#define ASSET(x)                                                                                                       \
    extern "C" {                                                                                                       \
    extern uint8_t _binary_static_##x##_json_start[], _binary_static_##x##_json_size[];                                          \
    static asset x = {_binary_static_##x##_json_start, (size_t)_binary_static_##x##_json_size};                                  \
    }

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
    static Path load(const std::asset& fileName);
};
