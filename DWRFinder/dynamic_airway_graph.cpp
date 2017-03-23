//
//  dynamic_airway_graph.cpp
//  DWR
//
//  Created by ZachQin on 2017/3/7.
//  Copyright © 2017年 Zach. All rights reserved.
//

#include "dynamic_airway_graph.hpp"

#include "dijkstra.hpp"
#include <set>

void DynamicAirwayGraph::GetDynamicPath(AirwayPointID sourceIdentity, AirwayPointID destinIdentity, std::vector<AirwayPoint> &path) {
    auto canSearch = [&](Edge edge, std::vector<Vertex> &previes) {
        return blockSet_.find(UndirectedEdge(edge)) == blockSet_.end();
    };
    GetPath(sourceIdentity, destinIdentity, path, canSearch);
}

void DynamicAirwayGraph::ForEachBlock(std::function<void (AirwayPoint, AirwayPoint)> &traverseFunction) {
    for (auto &blockEdge: blockSet_) {
        traverseFunction(airwayPointVector_[blockEdge.small], airwayPointVector_[blockEdge.big]);
    }
}
