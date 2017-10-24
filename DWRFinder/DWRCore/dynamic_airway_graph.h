//
//  dynamic_airway_graph.h
//  DWR
//
//  Created by ZachQin on 2017/3/7.
//  Copyright © 2017年 Zach. All rights reserved.
//

#ifndef dynamic_airway_graph_h
#define dynamic_airway_graph_h

#include "airway_graph.h"
#include <stdio.h>
#include <set>

namespace dwr {

struct UndirectedWaypointPair {
    std::shared_ptr<Waypoint> first;
    std::shared_ptr<Waypoint> second;
    
    UndirectedWaypointPair(const WaypointPair &pair) : first(std::min(pair.first, pair.second)), second(std::max(pair.first, pair.second)) {};
    UndirectedWaypointPair(const std::shared_ptr<Waypoint> &waypoint1, const std::shared_ptr<Waypoint> &waypoint2) : first(std::min(waypoint1, waypoint2)), second(std::max(waypoint1, waypoint2)) {};
        
    bool operator < (const UndirectedWaypointPair &p) const {
        return this->first < p.first ? true : (this->first > p.first ? false : this->second < p.second);
    };
};
    
class DynamicAirwayGraph: public AirwayGraph {
public:
    std::vector<std::shared_ptr<Waypoint>> GetDynamicPath(WaypointIdentifier origin_identifier, WaypointIdentifier destination_identifier);
    void ForEachBlock(std::function<void(const Waypoint &, const Waypoint &)> &traverse_function);
    
protected:
    std::set<UndirectedWaypointPair> block_set_;
};

}
#endif /* dynamic_airway_graph.h */
