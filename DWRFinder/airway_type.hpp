//
//  AirwayPoint.h
//  DWRFinder
//
//  Created by ZachQin on 2017/3/1.
//  Copyright © 2017年 Zach. All rights reserved.
//

#ifndef airway_type_hpp
#define airway_type_hpp

#include <string>
#include <vector>
#include <math.h>

namespace dwr {

typedef int AirwayPointID;
typedef double GeoDistance;
typedef double GeoRad;

struct AirwayPoint;
struct Neighbor {
    std::weak_ptr<AirwayPoint> target;
    GeoDistance distance;
    Neighbor() {};
    Neighbor(const std::shared_ptr<AirwayPoint> &arg_target, GeoDistance arg_distance)
    : target(arg_target), distance(arg_distance) { }
};
    
struct GeoPoint {
    GeoRad longitude;
    GeoRad latitude;
};
    
struct GeoProj {
    GeoDistance x;
    GeoDistance y;
};
    
const AirwayPointID kNoAirwaypointID = -1;
const GeoDistance kEarthRadius = 6378137.0;
const GeoDistance kNoCoordinate = std::numeric_limits<GeoDistance>::infinity();
    
struct AirwayPoint {
    AirwayPointID airwayPointID;
    std::string name;
    GeoPoint location;
    
    bool userWaypoint = false;
    std::vector<Neighbor> neibors;
    
    AirwayPoint() {};
    AirwayPoint(int airwayPointID, std::string name, double lon, double lat) : airwayPointID(airwayPointID), name(name), location({lon, lat}) {};
    
    bool operator < (const AirwayPoint &p) const {
        return airwayPointID < p.airwayPointID;
    }
    
    // Cache
    std::weak_ptr<AirwayPoint> previous;
    GeoDistance actualDistance = std::numeric_limits<GeoDistance>::max();
    GeoDistance heuristicDistance = std::numeric_limits<GeoDistance>::max();
    GeoProj coordinate = {kNoCoordinate, kNoCoordinate};
    
    void ResetCache() {
        actualDistance = std::numeric_limits<GeoDistance>::max();
        heuristicDistance = std::numeric_limits<GeoDistance>::max();
        previous.reset();
    };
    
    static GeoDistance Distance(const AirwayPoint &p1, const AirwayPoint &p2) {
        double FI1 = p1.location.latitude;
        double FI2 = p2.location.latitude;
        double deltFI = FI2 - FI1;
        double deltLamda = p2.location.longitude - p1.location.longitude;
        double a = sin(deltFI/2) * sin(deltFI/2) + cos(FI1)*cos(FI2) * sin(deltLamda/2) * sin(deltLamda/2);
        double c = 2 * atan2(sqrt(a), sqrt(1-a));
        return kEarthRadius * c;
    };
};
} //Namespace dwr

#endif /* AirwayPoint_h */
