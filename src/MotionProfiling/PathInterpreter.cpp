#include "PathInterpreter.h"
#include <iostream>
#include <fstream>
#include <sstream>

Path PathLoader::load(const std::string& file) {
    Path out;

    std::ifstream f(file);
    if (!f.is_open()) {
        std::cerr << "Failed to open path file: " << file << "\n";
        return out;
    }

    // Read entire file content
    std::ostringstream buf;
    buf << f.rdbuf();
    std::string content = buf.str();
    f.close();

    // Parse JSON with json11
    std::string err;
    json11::Json j = json11::Json::parse(content, err);
    if (!err.empty()) {
        std::cerr << "JSON parse error: " << err << "\n";
        return out;
    }

    // Load start/end speeds
    out.startSpeed = j["start_speed"].number_value();
    out.endSpeed = j["end_speed"].number_value();

    // Load segments
    for (auto& s : j["segments"].array_items()) {
        auto& p = s["path"];

        BezierSegment seg;
        seg.p0 = {p[0]["x"].number_value(), p[0]["y"].number_value()};
        seg.p1 = {p[1]["x"].number_value(), p[1]["y"].number_value()};
        seg.p2 = {p[2]["x"].number_value(), p[2]["y"].number_value()};
        seg.p3 = {p[3]["x"].number_value(), p[3]["y"].number_value()};
        seg.maxVel = s["constraints"]["velocity"].number_value();
        seg.maxAccel = s["constraints"]["accel"].number_value();
        seg.reversed = s["inverted"].bool_value();

        out.segments.push_back(seg);
    }

    // Load events
    for (auto& e : j["commands"].array_items()) {
        Event ev;
        ev.t = e["t"].number_value();
        ev.name = e["name"].string_value();
        out.events.push_back(ev);
    }

    return out;
}
