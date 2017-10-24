//
//  dynamic_airway_graph.cpp
//  DWR
//
//  Created by ZachQin on 2017/3/7.
//  Copyright © 2017年 Zach. All rights reserved.
//

#include "dynamic_airway_graph.hpp"
#include <set>

namespace dwr {
    
std::vector<std::shared_ptr<Waypoint>> DynamicAirwayGraph::GetDynamicPath(WaypointID originIdentifier, WaypointID destinIdentifier) {
    auto canSearch = [&](const WaypointPair &pair, std::vector<std::shared_ptr<Waypoint>> &insertedWaypoints) {
        // 90° limit
        if (blockSet_.find(UndirectedWaypointPair(pair)) == blockSet_.end()) {
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
    return GetPath(originIdentifier, destinIdentifier, canSearch);
}

void DynamicAirwayGraph::ForEachBlock(std::function<void (const Waypoint &, const Waypoint &)> &traverseFunction) {
    for (auto &block: blockSet_) {
        traverseFunction(*block.first, *block.second);
    }
}

}
