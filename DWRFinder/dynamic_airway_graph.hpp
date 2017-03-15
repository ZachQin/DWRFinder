//
//  dynamic_airway_graph.hpp
//  DWR
//
//  Created by ZachQin on 2017/3/7.
//  Copyright © 2017年 Zach. All rights reserved.
//

#ifndef dynamic_airway_graph_hpp
#define dynamic_airway_graph_hpp

#include "airway_graph.hpp"
#include <stdio.h>
#include <set>

class DynamicAirwayGraph: public AirwayGraph {
protected:
    std::set<Edge> blockSet_;
public:
    void GetDynamicPath(AirwayPointID sourceIdentity, AirwayPointID destinIdentity, std::vector<AirwayPoint> &path);
    void ForEachBlock(std::function<void(AirwayPoint, AirwayPoint)> &traverseFunction);
};

#endif /* dynamic_airway_graph_hpp */
