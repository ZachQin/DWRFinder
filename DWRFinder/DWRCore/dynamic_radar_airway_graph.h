//
//  dynamic_radar_airway_graph.h
//  DWRFinder
//
//  Created by ZachQin on 2017/3/8.
//  Copyright © 2017年 Zach. All rights reserved.
//

#ifndef dynamic_radar_airway_graph_h
#define dynamic_radar_airway_graph_h

#include <stdio.h>
#include <map>
#include "dynamic_airway_graph.h"
#include "graphics_utils.h"
#include "raster_graph.h"

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
    /**
     Build all waypoint with world file.

     @param world_file_info world file.
     */
    void Prebuild(const WorldFileInfo &world_file_info);
    
    /**
     Build single waypoint if necessary after prebuilding.

     @param identifier waypoint ID.
     */
    void Build(WaypointIdentifier identifier);
    // mask包含0或1，1表示有阻塞，0表示无阻塞
    void UpdateBlock(const std::shared_ptr<const char> &mask, int width, int height);
    std::vector<std::shared_ptr<Waypoint>> GetDynamicFullPath(WaypointIdentifier origin_identifier, WaypointIdentifier destination_identifier);
    //Debug
//    void LogBlockAirpointSegment();

private:
    std::map<Pixel, UndirectedWaypointPair> pixel_to_edge_table_;
    RasterGraph raster_graph_;
    WorldFileInfo world_file_info_;
};
    
}
#endif /* dynamic_radar_airway_graph_h */
