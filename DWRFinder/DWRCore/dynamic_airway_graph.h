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
#include <set>

namespace dwr {
    
struct UndirectedWaypointPair : public WaypointPair {
    UndirectedWaypointPair(const WaypointPair &pair) : WaypointPair(std::min(pair.first, pair.second), std::max(pair.first, pair.second)) {};
    UndirectedWaypointPair(const std::shared_ptr<Waypoint> &waypoint1, const std::shared_ptr<Waypoint> &waypoint2) : WaypointPair(std::min(waypoint1, waypoint2), std::max(waypoint1, waypoint2)) {};
};
    
class DynamicAirwayGraph: public AirwayGraph {
public:
    std::vector<std::shared_ptr<Waypoint>> FindDynamicPath(WaypointIdentifier origin_identifier, WaypointIdentifier destination_identifier);
    void ForEachBlock(std::function<void(const Waypoint &, const Waypoint &)> &traverse_function);
    
protected:
    std::set<UndirectedWaypointPair> block_set_;
};

}
#endif /* dynamic_airway_graph.h */
