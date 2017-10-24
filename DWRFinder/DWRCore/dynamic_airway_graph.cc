//
//  dynamic_airway_graph.cc
//  DWR
//
//  Created by ZachQin on 2017/3/7.
//  Copyright © 2017年 Zach. All rights reserved.
//

#include "dynamic_airway_graph.h"
#include <set>

namespace dwr {
    
std::vector<std::shared_ptr<Waypoint>> DynamicAirwayGraph::GetDynamicPath(WaypointIdentifier origin_identifier, WaypointIdentifier destination_identifier) {
    auto can_search = [&](const WaypointPair &pair, std::vector<std::shared_ptr<Waypoint>> &inserted_waypoints) {
        // 90° limit
        if (block_set_.find(UndirectedWaypointPair(pair)) == block_set_.end()) {
            if (pair.first->previous.lock() == nullptr) {
                return true;
            } else if (CosinTurnAngle(pair.first->previous.lock(), pair.first, pair.second) > 0) {
                return true;
            } else {
                return false;
            }
        } else {
            return false;
        }
    };
    return GetPath(origin_identifier, destination_identifier, can_search);
}

void DynamicAirwayGraph::ForEachBlock(std::function<void (const Waypoint &, const Waypoint &)> &traverse_function) {
    for (auto &block: block_set_) {
        traverse_function(*block.first, *block.second);
    }
}

}
