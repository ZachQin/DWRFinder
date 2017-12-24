//
//  airway_type.cc
//  DWRFinder
//
//  Created by ZachQin on 2017/12/19.
//  Copyright © 2017年 Zach. All rights reserved.
//

#include "airway_type.h"

#include <cmath>

namespace dwr {

GeoDistance Waypoint::Distance(const dwr::Waypoint &p1,
                               const dwr::Waypoint &p2) {
    double FI1 = p1.location.latitude;
    double FI2 = p2.location.latitude;
    double deltFI = FI2 - FI1;
    double deltLamda = p2.location.longitude - p1.location.longitude;
    double a = sin(deltFI/2) * sin(deltFI/2) + cos(FI1)*cos(FI2) * sin(deltLamda/2) * sin(deltLamda/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return kEarthRadius * c;
}

double Waypoint::CosinTurnAngle(const dwr::Waypoint &previous,
                                const dwr::Waypoint &current,
                                const dwr::Waypoint &next) {
    if (previous.coordinate == kNoCoordinate ||
        current.coordinate == kNoCoordinate ||
        next.coordinate == kNoCoordinate) {
        throw std::invalid_argument("no coordinate");
    }
    double pc_x = current.coordinate.x - previous.coordinate.x;
    double pc_y = current.coordinate.y - previous.coordinate.y;
    double cn_x = next.coordinate.x - current.coordinate.x;
    double cn_y = next.coordinate.y - current.coordinate.y;
    return (pc_x * cn_x + pc_y * cn_y) / (sqrt(pc_x * pc_x + pc_y * pc_y) * sqrt(cn_x * cn_x + cn_y * cn_y));
}

WaypointPath::WaypointPath(const WaypointPath &other, int start, int node_count) {
    waypoints.reserve(node_count);
    lengths.reserve(node_count);
    waypoints.insert(waypoints.end(), other.waypoints.begin() + start, other.waypoints.begin() + start + node_count);
    lengths.insert(lengths.end(), other.lengths.begin() + start, other.lengths.begin() + start + node_count);
    std::for_each(lengths.begin(), lengths.end(), [&](GeoDistance &distance){distance -= lengths[0];});
}

WaypointPath WaypointPath::operator+(const dwr::WaypointPath &path) const {
    if (waypoints.back() != path.waypoints.front()) {
        throw std::invalid_argument("back of left path not same as front of right path");
    }
    WaypointPath result(*this, 0, GetSize());
    int sum_size = GetSize() + path.GetSize() - 1;
    result.waypoints.reserve(sum_size);
    result.lengths.reserve(sum_size);
    result.waypoints.insert(result.waypoints.end(), path.waypoints.begin() + 1, path.waypoints.end());
    result.lengths.insert(result.lengths.end(), path.lengths.begin() + 1, path.lengths.end());
    std::for_each(result.lengths.begin() + GetSize(), result.lengths.end(), [&](GeoDistance &distance){distance += lengths.back();});
    return result;
}

std::string WaypointPath::ToString() const {
    std::string path_description;
    if (waypoints.empty()) {
        return path_description;
    }
    for (auto it = waypoints.begin(); it < waypoints.end() - 1; it++) {
        path_description += (*it)->name;
        path_description += "->";
    }
    path_description += waypoints.back()->name;
    return path_description;
}

double WaypointPath::GetSumTurn() const {
    double sum_turn = 0.0;
    for (size_t i = 0; i < waypoints.size() - 2; i++) {
        double cos_turn = Waypoint::CosinTurnAngle(*waypoints[i], *waypoints[i + 1], *waypoints[i + 2]);
        sum_turn += acos(cos_turn);
    }
    return sum_turn;
}

}  // namespace dwr
