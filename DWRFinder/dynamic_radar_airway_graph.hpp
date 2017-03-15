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

struct WorldFileInfo {
    double A;
    double D;
    double B;
    double E;
    double C;
    double F;
    
    WorldFileInfo(const char *path);
};

struct Pixel {
    int x;
    int y;
    
    Pixel() {};
    Pixel(int x, int y): x(x), y(y) {};
    
    bool operator < (const Pixel &p) const {
        return this->x < p.x ? true : (this->x > p.x ? false : this->y < p.y);
    }
};

class DynamicRadarAirwayGraph: public DynamicAirwayGraph {
    std::map<Pixel, Edge> pixelToEdgeTable_;
    
public:
    void prebuild(const WorldFileInfo &worldFileInfo);
    // mask包含0或1，1表示有阻塞，0表示无阻塞
    void UpdateBlock(const char *mask, size_t width, size_t height);
    
    //Debug
//    void LogBlockAirpointSegment();
};
#endif /* dynamic_radar_airway_graph_hpp */
