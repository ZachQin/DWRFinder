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

namespace dwr {

class DynamicAirwayGraph: public AirwayGraph {
public:
    void GetDynamicPath(AirwayPointID sourceIdentity, AirwayPointID destinIdentity, std::vector<AirwayPoint> &path);
    void ForEachBlock(std::function<void(AirwayPoint, AirwayPoint)> &traverseFunction);
    
protected:
    std::set<UndirectedEdge> blockSet_;
};

}
#endif /* dynamic_airway_graph_hpp */
