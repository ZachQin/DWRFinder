//
//  AirwayPoint.h
//  DWRFinder
//
//  Created by ZachQin on 2017/3/1.
//  Copyright © 2017年 Zach. All rights reserved.
//

#include <string>
#include <math.h>

#ifndef airway_type_h
#define airway_type_h

typedef int32_t AirwayPointID;

struct AirwayPoint {
    AirwayPointID airwayPointID;
    std::string name;
    double x;
    double y;
    
    double longitude;
    double latitude;
    
    AirwayPoint() {};
    AirwayPoint(int airwayPointID, std::string name, double x, double y, double lon, double lat):airwayPointID(airwayPointID), name(name), x(x), y(y), longitude(lon), latitude(lat) {};
    
    double distance(AirwayPoint &to) {
        return sqrt((this->x - to.x) * (this->x - to.x) + (this->y - to.y) * (this->y - to.y));
    }
}__attribute__ ((aligned (8)));;

#endif /* AirwayPoint_h */
