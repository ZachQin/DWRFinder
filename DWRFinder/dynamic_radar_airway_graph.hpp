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
#include "dynamic_airway_graph.hpp"
#include "graphics_utils.hpp"
#include "raster_graph.hpp"

namespace dwr {

/**
 A world file is a six line plain text sidecar file used by geographic information systems (GIS) to georeference raster map images. The file specification was introduced by Esri,[1][2] and consists of six coefficients of an affine transformation that describes the location, scale and rotation of a raster on a map.
 */
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
public:
    void prebuild(const WorldFileInfo &worldFileInfo);
    // mask包含0或1，1表示有阻塞，0表示无阻塞
    void UpdateBlock(const std::shared_ptr<const char> &mask, int width, int height);
    std::vector<std::shared_ptr<Waypoint>> GetDynamicFullPath(WaypointID originIdentity, WaypointID destinIdentity);
    //Debug
//    void LogBlockAirpointSegment();

private:
    std::map<Pixel, UndirectedWaypointPair> pixelToEdgeTable_;
    RasterGraph rasterGraph_;
    WorldFileInfo worldFileInfo_;
};
    
}
#endif /* dynamic_radar_airway_graph_hpp */
