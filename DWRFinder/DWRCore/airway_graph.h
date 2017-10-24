//
//  AirwayGraph.h
//  DWR
//
//  Created by ZachQin on 2017/3/3.
//  Copyright © 2017年 Zach. All rights reserved.
//

#ifndef airway_graph_h
#define airway_graph_h

#include <stdio.h>
#include <map>
#include <vector>
#include "airway_type.h"

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
     @param longitude Waypoint longitude.
     @param latitude Waypoint latitude.
     */
    void AddWaypoint(WaypointIdentifier identifier, const std::string &name, GeoRad longitude, GeoRad latitude);
    
    /**
     Remove a waypoint and all of its connections.

     @param identifier Waypoint ID;
     */
    void RemoveWaypoint(WaypointIdentifier identifier);
    
    /**
     Add the connection between two waypoint.
     
     @param identifier1 First waypoint.
     @param identifier2 Second waypoint.
     */
    void AddAirwaySegment(WaypointIdentifier identifier1, WaypointIdentifier identifier2);
    
    /**
     Get the path using A* algorithm.
     
     @param origin_identifier Origin waypoint ID
     @param destination_identifier Destination waypoint ID
     @param can_search The Lambda expression using to determine whether the edge can be access.
     @return The shortest path.
     */
    std::vector<std::shared_ptr<Waypoint>> GetPath(WaypointIdentifier origin_identifier, WaypointIdentifier destination_identifier, const std::function<bool(const WaypointPair &, std::vector<std::shared_ptr<Waypoint>> &)> can_search = [](const WaypointPair &p, std::vector<std::shared_ptr<Waypoint>> &inserted_waypoints){return true;});
    
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
     
     @param  traverse_function The Lambda expression applied to all edge in the graph.
     */
    void ForEach(std::function<void(const std::shared_ptr<Waypoint> &, const std::shared_ptr<Waypoint> &, GeoDistance)> traverse_function);
    
    /**
     Get All WaypointID

     @return Waypoints' ID vector.
     */
    std::vector<WaypointIdentifier> AllWaypointIdentifiers();
    
    /**
     Get waypoint from waypoint ID.
     
     @param identifier Waypoint ID.
     @return Waypoint pointer.
     */
    std::shared_ptr<Waypoint> WaypointFromIdentifier(WaypointIdentifier identifier);

protected:
    std::map<WaypointIdentifier, std::shared_ptr<Waypoint>> waypoint_map_;
};

double CosinTurnAngle(const std::shared_ptr<Waypoint> &previous, const std::shared_ptr<Waypoint> &current, const std::shared_ptr<Waypoint> &next);
    
}
#endif /* AirwayGraph_h */
