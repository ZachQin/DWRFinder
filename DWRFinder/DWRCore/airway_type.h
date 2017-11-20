//
//  Waypoint.h
//  DWRFinder
//
//  Created by ZachQin on 2017/3/1.
//  Copyright © 2017年 Zach. All rights reserved.
//

#ifndef airway_type_h
#define airway_type_h

#include <math.h>

#include <string>
#include <set>
#include <memory>
#include <limits>
#include <vector>

namespace dwr {

typedef int WaypointIdentifier;
typedef double GeoDistance;
typedef double GeoRad;

struct Waypoint;
struct Neighbor {
    std::weak_ptr<Waypoint> target;
    GeoDistance distance;

    Neighbor() {}

    Neighbor(const std::shared_ptr<Waypoint> &arg_target, GeoDistance arg_distance) :
    target(arg_target), distance(arg_distance) {}

    bool operator < (const Neighbor &p) const {
        return target.lock() < p.target.lock();
    }
};

const WaypointIdentifier kNoWaypointIdentifier = -1;
const GeoDistance kEarthRadius = 6378137.0;

struct GeoPoint {
    GeoRad longitude;
    GeoRad latitude;
};

struct GeoProj {
    GeoDistance x;
    GeoDistance y;

    bool operator == (const GeoProj &other) const {
        return x == other.x && y == other.y;
    }
};

constexpr GeoProj kNoCoordinate = {
    std::numeric_limits<GeoDistance>::infinity(),
    std::numeric_limits<GeoDistance>::infinity()};

struct Waypoint {
    WaypointIdentifier waypoint_identifier;
    std::string name;
    GeoPoint location;
    GeoProj coordinate = kNoCoordinate;

    bool user_waypoint = false;
    std::set<Neighbor> neibors;

    Waypoint() {}

    Waypoint(const Waypoint &other) :
    waypoint_identifier(other.waypoint_identifier),
    name(other.name), location(other.location), coordinate(other.coordinate),
    user_waypoint(other.user_waypoint) {}

    Waypoint(int waypoint_identifier, const std::string &name, double lon, double lat) :
    waypoint_identifier(waypoint_identifier), name(name), location({lon, lat}) {}

    bool operator < (const Waypoint &p) const {
        return waypoint_identifier < p.waypoint_identifier;
    }

    static GeoDistance Distance(const Waypoint &p1, const Waypoint &p2) {
        double FI1 = p1.location.latitude;
        double FI2 = p2.location.latitude;
        double deltFI = FI2 - FI1;
        double deltLamda = p2.location.longitude - p1.location.longitude;
        double a = sin(deltFI/2) * sin(deltFI/2) + cos(FI1)*cos(FI2) * sin(deltLamda/2) * sin(deltLamda/2);
        double c = 2 * atan2(sqrt(a), sqrt(1-a));
        return kEarthRadius * c;
    }

    static double CosinTurnAngle(const Waypoint &previous, const Waypoint &current, const Waypoint &next) {
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
};

struct WaypointInfo {
    std::weak_ptr<const Waypoint> previous;
    GeoDistance actual_distance = std::numeric_limits<GeoDistance>::max();
    GeoDistance heuristic_distance = std::numeric_limits<GeoDistance>::max();
};

struct WaypointPath {
    std::vector<std::shared_ptr<const Waypoint>> waypoints;
    std::vector<GeoDistance> lengths;

    WaypointPath() = default;

    WaypointPath(const WaypointPath &other, int start, int node_count) {
        waypoints.reserve(node_count);
        lengths.reserve(node_count);
        waypoints.insert(waypoints.end(), other.waypoints.begin() + start, other.waypoints.begin() + start + node_count);
        lengths.insert(lengths.end(), other.lengths.begin() + start, other.lengths.begin() + start + node_count);
        std::for_each(lengths.begin(), lengths.end(), [&](GeoDistance &distance){distance -= lengths[0];});
    }

    int GetSize() const {return static_cast<int>(waypoints.size());}

    WaypointPath operator+ (const WaypointPath &path) const {
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

    std::string ToString() const {
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
};

}  // namespace dwr

#endif /* Waypoint_h */
