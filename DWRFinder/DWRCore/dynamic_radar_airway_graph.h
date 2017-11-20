//
//  dynamic_radar_airway_graph.h
//  DWRFinder
//
//  Created by ZachQin on 2017/3/8.
//  Copyright © 2017年 Zach. All rights reserved.
//

#ifndef dynamic_radar_airway_graph_h
#define dynamic_radar_airway_graph_h

#include <unordered_map>
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
     SingleBuild all waypoint with world file.

     @param world_file_info World file.
     */
    void Build(const WorldFileInfo &world_file_info);
    
    /**
     SingleBuild single waypoint if necessary after prebuilding.

     @param identifier Waypoint ID.
     */
    void SingleBuild(WaypointIdentifier identifier);
    /**
     Update the mask.

     @param mask Bitmap that 1 means block and 0 means non-block.
     @param width Width of mask pixel
     @param height Height of mask pixel
     */
    void UpdateBlock(char *mask, int width, int height);
    /**
     Find path with double scale A* search.

     @param origin_identifier Origin waypoint identifier.
     @param destination_identifier Destination waypoint identifier.
     @return Path consists of waypoints.
     */
    WaypointPath
    FindDynamicFullPath(WaypointIdentifier origin_identifier,
                        WaypointIdentifier destination_identifier,
                        const std::function<bool(const WaypointPair &waypoint_pair,
                                                 const WaypointInfoPair &info_pair,
                                                 std::vector<std::shared_ptr<Waypoint>> &inserted_waypoints)> &can_search
                        = [](const WaypointPair &,
                             const WaypointInfoPair &,
                             std::vector<std::shared_ptr<Waypoint>> &inserted_waypoints) {return true;}
                        ) const;
    
    std::vector<WaypointPath>
    FindKDynamicFullPath(WaypointIdentifier origin_identifier,
                         WaypointIdentifier destination_identifier,
                         int k) const;

private:
    std::unordered_map<Pixel, UndirectedWaypointPair> pixel_to_edge_table_;
    RasterGraph raster_graph_;
    WorldFileInfo world_file_info_;
};
    
}
#endif /* dynamic_radar_airway_graph_h */
