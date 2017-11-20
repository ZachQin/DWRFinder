//
//  dynamic_airway_graph.cc
//  DWR
//
//  Created by ZachQin on 2017/3/7.
//  Copyright © 2017年 Zach. All rights reserved.
//

#include "dynamic_airway_graph.h"

#include <memory>
#include <vector>

namespace dwr {

WaypointPath
DynamicAirwayGraph::FindDynamicPath(WaypointIdentifier origin_identifier,
                                    WaypointIdentifier destination_identifier) const {
    auto can_search = [&](const WaypointPair &waypoint_pair,
                          const WaypointInfoPair &info_pair,
                          std::vector<WaypointPtr> &inserted_waypoints) {
        // False when waypoint pair in block set.
        if (block_set_.find(UndirectedWaypointPair(waypoint_pair)) == block_set_.end()) {
            // 90° limit
            if (info_pair.first.previous.lock() == nullptr) {
                return true;
            } else if (Waypoint::CosinTurnAngle(*info_pair.first.previous.lock(),
                                                *waypoint_pair.first,
                                                *waypoint_pair.second) > 0) {
                return true;
            } else {
                return false;
            }
        } else {
            return false;
        }
    };
    return FindPath(origin_identifier, destination_identifier, can_search);
}

void
DynamicAirwayGraph::ForEachBlock(const std::function<void(const Waypoint &,
                                                          const Waypoint &)>
                                 &traverse_function) const {
    for (auto &block : block_set_) {
        traverse_function(*block.first, *block.second);
    }
}

}  // namespace dwr
