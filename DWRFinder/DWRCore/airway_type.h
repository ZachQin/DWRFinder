//
//  Waypoint.h
//  DWRFinder
//
//  Created by ZachQin on 2017/3/1.
//  Copyright © 2017年 Zach. All rights reserved.
//

#ifndef airway_type_h
#define airway_type_h

#include <string>
#include <memory>
#include <limits>
#include <vector>

namespace dwr {

using WaypointIdentifier = int;
using GeoDistance = double;
using GeoRad = double;

struct Waypoint;

using WaypointPtr = std::shared_ptr<Waypoint>;
using ConstWaypointPtr = std::shared_ptr<const Waypoint>;

struct Neighbor {
    std::weak_ptr<Waypoint> target;
    GeoDistance distance;

    Neighbor() {}

    Neighbor(const WaypointPtr &arg_target, GeoDistance arg_distance) :
    target(arg_target), distance(arg_distance) {}
    
    bool operator == (const Neighbor &n) const {
        return target.lock() == n.target.lock();
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
        return std::tie(x, y) == std::tie(other.x, other.y);
    }
};

constexpr GeoProj kNoCoordinate = {
    std::numeric_limits<GeoDistance>::infinity(),
    std::numeric_limits<GeoDistance>::infinity()};

struct Waypoint {
    WaypointIdentifier identifier;
    std::string name;
    GeoPoint location;
    GeoProj coordinate = kNoCoordinate;

    bool user_waypoint = false;
    std::vector<Neighbor> neibors;

    Waypoint() {}

    Waypoint(const Waypoint &other) :
    identifier(other.identifier),
    name(other.name), location(other.location), coordinate(other.coordinate),
    user_waypoint(other.user_waypoint) {}

    Waypoint(int waypoint_identifier, const std::string &name, double lon, double lat) :
    identifier(waypoint_identifier), name(name), location({lon, lat}) {}

    bool operator < (const Waypoint &p) const {
        return identifier < p.identifier;
    }

    static GeoDistance Distance(const Waypoint &p1,
                                const Waypoint &p2);

    static double CosinTurnAngle(const Waypoint &previous,
                                 const Waypoint &current,
                                 const Waypoint &next);
};

struct WaypointInfo {
    std::weak_ptr<const Waypoint> previous;
    GeoDistance actual_distance = std::numeric_limits<GeoDistance>::max();
    GeoDistance estimated_distance = std::numeric_limits<GeoDistance>::max();
};

struct WaypointPath {
    std::vector<ConstWaypointPtr> waypoints;
    std::vector<GeoDistance> lengths;

    WaypointPath() = default;

    WaypointPath(const WaypointPath &other, int start, int node_count);

    int GetSize() const {return static_cast<int>(waypoints.size());}

    double GetSumTurn() const;

    WaypointPath operator+ (const WaypointPath &path) const;

    std::string ToString() const;
};

}  // namespace dwr

#endif /* Waypoint_h */
