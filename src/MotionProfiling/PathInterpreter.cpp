#include "PathInterpreter.h"
#include <iostream>
#include <fstream>
#include <sstream>

Path PathLoader::load(const std::asset& file) {
    Path path;

    std::string content(reinterpret_cast<const char*>(file.data), file.size);
    std::string err;
    json11::Json parsedJson = json11::Json::parse(content, err);
    
    if (!err.empty()) {
        return path;
    }

    path.startSpeed = parsedJson["start_speed"].number_value();
    path.endSpeed = parsedJson["end_speed"].number_value();

    for (auto& s : parsedJson["segments"].array_items()) {
        auto& p = s["path"];

        BezierSegment seg;
        seg.p0 = {p[0]["x"].number_value(), p[0]["y"].number_value()};
        seg.p1 = {p[1]["x"].number_value(), p[1]["y"].number_value()};
        seg.p2 = {p[2]["x"].number_value(), p[2]["y"].number_value()};
        seg.p3 = {p[3]["x"].number_value(), p[3]["y"].number_value()};
        seg.maxVel = s["constraints"]["velocity"].number_value();
        seg.maxAccel = s["constraints"]["accel"].number_value();
        seg.reversed = s["inverted"].bool_value();

        path.segments.push_back(seg);
    }

    
    for (auto& e : parsedJson["commands"].array_items()) {
        Event ev;
        ev.t = e["t"].number_value();
        ev.name = e["name"].string_value();
        path.events.push_back(ev);
    }

    return path;
}
