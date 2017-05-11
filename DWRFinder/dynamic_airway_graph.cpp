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
        return blockSet_.find(UndirectedWaypointPair(pair)) == blockSet_.end();
    };
    return GetPath(originIdentifier, destinIdentifier, canSearch);
}

void DynamicAirwayGraph::ForEachBlock(std::function<void (const Waypoint &, const Waypoint &)> &traverseFunction) {
    for (auto &block: blockSet_) {
        traverseFunction(*block.first, *block.second);
    }
}

}
