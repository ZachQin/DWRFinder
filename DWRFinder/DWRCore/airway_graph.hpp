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
#include <vector>
#include "airway_type.hpp"

namespace dwr {

typedef std::pair<const std::shared_ptr<Waypoint>, const std::shared_ptr<Waypoint>> WaypointPair;
    
class AirwayGraph {
public:
    AirwayGraph(){};
    AirwayGraph(const char *path);
    
    /**
     Add a waypoint to the graph.
     
     @param identifier Waypoint ID.
     @param name Waypoint name.
     @param lon Waypoint longitude.
     @param lat Waypoint latitude.
     */
    void AddWaypoint(WaypointID identifier, const std::string &name, GeoRad lon, GeoRad lat);
    
    /**
     Remove a waypoint and all of its connections.

     @param identifier Waypoint ID;
     */
    void RemoveWaypoint(WaypointID identifier);
    
    /**
     Add the connection between two waypoint.
     
     @param identifier1 First waypoint.
     @param identifier2 Second waypoint.
     */
    void AddAirwaySegment(WaypointID identifier1, WaypointID identifier2);
    
    /**
     Get the path using A* algorithm.
     
     @param originIdentifier Origin waypoint ID
     @param destinIdentifier Destination waypoint ID
     @param canSearch The Lambda expression using to determine whether the edge can be access.
     @return The shortest path.
     */
    std::vector<std::shared_ptr<Waypoint>> GetPath(WaypointID originIdentifier, WaypointID destinIdentifier, const std::function<bool(const WaypointPair &, std::vector<std::shared_ptr<Waypoint>> &)> canSearch = [](const WaypointPair &p, std::vector<std::shared_ptr<Waypoint>> &insertedWaypoints){return true;});
    
    /**
     Save the graph as a file.
     
     @param path File path.
     @return True when succeed, otherwise false.
     */
    bool SaveToFile(const std::string &path) const;
    
    /**
     Load the graph from a file.
     
     @param path File path.
     @return True when succeed, otherwise false.
     */
    bool LoadFromFile(const std::string &path);
    /**
     Applies the given Lambda expression to all edge in the graph.
     
     @param  traverseFunction The Lambda expression applied to all edge in the graph.
     */
    void ForEach(std::function<void(const std::shared_ptr<Waypoint> &, const std::shared_ptr<Waypoint> &, GeoDistance)> traverseFunction);
    
    /**
     Get All WaypointID

     @return Waypoints' ID vector.
     */
    std::vector<WaypointID> AllWaypointID();
    
    /**
     Get waypoint from waypoint ID.
     
     @param identifier Waypoint ID.
     @return Waypoint pointer.
     */
    std::shared_ptr<Waypoint> WaypointFromID(WaypointID identifier);

protected:
    std::map<WaypointID, std::shared_ptr<Waypoint>> waypointMap_;
};

double CosinTurnAngle(const std::shared_ptr<Waypoint> &previous, const std::shared_ptr<Waypoint> &current, const std::shared_ptr<Waypoint> &next);
    
}
#endif /* AirwayGraph_hpp */
