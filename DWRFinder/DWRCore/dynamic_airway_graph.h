//
//  dynamic_airway_graph.h
//  DWR
//
//  Created by ZachQin on 2017/3/7.
//  Copyright © 2017年 Zach. All rights reserved.
//

#ifndef dynamic_airway_graph_h
#define dynamic_airway_graph_h

#include <set>
#include <memory>
#include <algorithm>

#include "airway_graph.h"

namespace dwr {

struct UndirectedWaypointPair : public WaypointPair {
    explicit UndirectedWaypointPair(const WaypointPair &pair) :
    WaypointPair(std::min(pair.first, pair.second), std::max(pair.first, pair.second)) {}

    UndirectedWaypointPair(const WaypointPtr &waypoint1,
                           const WaypointPtr &waypoint2) :
    WaypointPair(std::min(waypoint1, waypoint2), std::max(waypoint1, waypoint2)) {}
};

class DynamicAirwayGraph: public AirwayGraph {
 public:
    WaypointPath FindDynamicPath(WaypointIdentifier origin_identifier,
                                 WaypointIdentifier destination_identifier) const;

    void ForEachBlock(const std::function<void(const Waypoint &, const Waypoint &)> &traverse_function) const;
 protected:
    std::set<UndirectedWaypointPair> block_set_;
};

}  // namespace dwr
#endif /* dynamic_airway_graph.h */
