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
    
std::vector<std::shared_ptr<AirwayPoint>> DynamicAirwayGraph::GetDynamicPath(AirwayPointID sourceIdentity, AirwayPointID destinIdentity) {
    auto canSearch = [&](const AirwayPointPair &pair) {
        return blockSet_.find(UndirectedAirwayPointPair(pair)) == blockSet_.end();
    };
    return GetPath(sourceIdentity, destinIdentity, canSearch);
}

void DynamicAirwayGraph::ForEachBlock(std::function<void (const AirwayPoint &, const AirwayPoint &)> &traverseFunction) {
    for (auto &block: blockSet_) {
        traverseFunction(*block.first, *block.second);
    }
}

}
