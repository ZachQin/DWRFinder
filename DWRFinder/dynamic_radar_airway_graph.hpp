//
//  dynamic_radar_airway_graph.hpp
//  DWRFinder
//
//  Created by ZachQin on 2017/3/8.
//  Copyright © 2017年 Zach. All rights reserved.
//

#ifndef dynamic_radar_airway_graph_hpp
#define dynamic_radar_airway_graph_hpp

#include <stdio.h>
#include <map>
#include "graph.h"
#include "dynamic_airway_graph.hpp"
#include "graphics_utils.hpp"

struct WorldFileInfo {
    double A;
    double D;
    double B;
    double E;
    double C;
    double F;
    WorldFileInfo() {};
    WorldFileInfo(const char *path);
};

class DynamicRadarAirwayGraph: public DynamicAirwayGraph {
    std::map<Pixel, Edge> pixelToEdgeTable_;
    const char *radarMask_;
    int radarWidth_;
    int radarHeight_;
    WorldFileInfo worldFileInfo_;
public:
    void prebuild(const WorldFileInfo &worldFileInfo);
    // mask包含0或1，1表示有阻塞，0表示无阻塞
    void UpdateBlock(const char *mask, int width, int height);
    void GetDynamicFullPath(AirwayPointID sourceIdentity, AirwayPointID destinIdentity, std::vector<AirwayPoint> &path);
    //Debug
//    void LogBlockAirpointSegment();
};
#endif /* dynamic_radar_airway_graph_hpp */
