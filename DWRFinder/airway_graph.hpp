//
//  AirwayGraph.hpp
//  DWR
//
//  Created by ZachQin on 2017/3/3.
//  Copyright © 2017年 Zach. All rights reserved.
//

#ifndef airway_graph_hpp
#define airway_graph_hpp

#include <stdio.h>
#include <map>
#include "graph.h"
#include "airway_type.hpp"

namespace dwr {

class AirwayGraph {
public:
    AirwayGraph(){};
    AirwayGraph(const char *path);
    void AddAirwayPoint(AirwayPointID identity, std::string name, double x, double y, double lon, double lat);
    void AddAirwaySegment(AirwayPointID identity1, AirwayPointID identity2);
    void GetPath(AirwayPointID sourceIdentity, AirwayPointID destinIdentity, std::vector<AirwayPoint> &path, const std::function<bool(Edge, std::vector<Vertex> &)> &canSearch = [](Edge edge, std::vector<Vertex> &previes){return true;});
    bool SaveToFile(const std::string &path);
    bool LoadFromFile(const std::string &path);
    void ForEach(std::function<void(AirwayPoint, AirwayPoint, double)> &traverseFunction);
    AirwayPoint AirwayPointFromID(AirwayPointID identity);

protected:
    std::map<AirwayPointID, Vertex> airwayPointMap_;
    // 下标是Vertex
    std::vector<AirwayPoint> airwayPointVector_;
    adjacency_list_t adjacencyList_;
};

}
#endif /* AirwayGraph_hpp */
