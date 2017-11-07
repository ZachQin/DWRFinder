//
//  AirwayGraph.h
//  DWR
//
//  Created by ZachQin on 2017/3/3.
//  Copyright © 2017年 Zach. All rights reserved.
//

#ifndef airway_graph_h
#define airway_graph_h

#include <map>
#include <vector>
#include <queue>
#include "airway_type.h"

namespace dwr {

typedef std::pair<const std::shared_ptr<Waypoint>, const std::shared_ptr<Waypoint>> WaypointPair;
typedef std::pair<const WaypointInfo, const WaypointInfo> WaypointInfoPair;
    
class AirwayGraph;
    
class AirwayGraph {
public:
    AirwayGraph() = default;
    
    AirwayGraph(const char *path);
    
    AirwayGraph(const AirwayGraph &other);
    
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

     @param identifier Waypoint identifier.
     */
    void RemoveWaypoint(WaypointIdentifier identifier);
    
    /**
     Add the connection between two waypoint.

     @param waypoint1 First waypoint pointer.
     @param waypoint2 Second waypoint pointer.
     */
    static void AddAirwaySegment(const std::shared_ptr<Waypoint> &waypoint1, const std::shared_ptr<Waypoint> &waypoint2);
    
    /**
     Add the connection between two waypoint.
     
     @param identifier1 First waypoint identifier.
     @param identifier2 Second waypoint identifier.
     */
    void AddAirwaySegment(WaypointIdentifier identifier1, WaypointIdentifier identifier2);
    
    /**
     Remove all of the waypoint's connections.
     
     @param waypoint Waypoint pointer.
     */
    static void RemoveAirwaySegments(const std::shared_ptr<Waypoint> &waypoint);
    
    /**
     Remove the connection between two waypoint.

     @param waypoint1 First waypoint pointer.
     @param waypoint2 Second waypoint pointer.
     */
    static void RemoveAirwaySegment(const std::shared_ptr<Waypoint> &waypoint1, const std::shared_ptr<Waypoint> &waypoint2);
    
    /**
     Remove the connection between two waypoint.

     @param identifier1 First waypoint identifier.
     @param identifier2 Second waypoint identifier.
     */
    void RemoveAirwaySegment(WaypointIdentifier identifier1, WaypointIdentifier identifier2);
    
    /**
     Get the path using A* algorithm.
     
     @param origin_identifier Origin waypoint ID
     @param destination_identifier Destination waypoint ID
     @param can_search The function using to determine whether the edge can be access.
     @return The shortest path.
     */
    WaypointPath
    FindPath(WaypointIdentifier origin_identifier,
             WaypointIdentifier destination_identifier,
             const std::function<bool(const WaypointPair &, const WaypointInfoPair &, std::vector<std::shared_ptr<Waypoint>> &)> &can_search
             = [](const WaypointPair &, const WaypointInfoPair &,
                  std::vector<std::shared_ptr<Waypoint>> &inserted_waypoints) {return true;}
             ) const;
    
    /**
     Get k shortest paths using Yen's algorithm.

     @param origin_identifier Origin waypoint identifier
     @param destination_identifier Destination waypoint identifier
     @param k Number of paths.
     @param find_path The function using to find a single path.
     @return The vector of shortest path.
     */

    std::vector<WaypointPath>
    FindKPath(WaypointIdentifier origin_identifier,
              WaypointIdentifier destination_identifier,
              int k,
              const std::function<WaypointPath (const std::shared_ptr<Waypoint> &spur_waypoint, const std::shared_ptr<Waypoint> &destination_waypoint)> &find_path
              ) const;
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
    void ForEach(const std::function<void(const std::shared_ptr<Waypoint> &waypoint1,
                                          const std::shared_ptr<Waypoint> &waypoint2,
                                          GeoDistance distance)> &traverse_function);
    
    /**
     Get All WaypointID

     @return Waypoints' ID vector.
     */
    std::vector<WaypointIdentifier> AllWaypointIdentifiers() const;
    
    /**
     Get waypoint from waypoint ID.
     
     @param identifier Waypoint ID.
     @return Waypoint pointer.
     */
    std::shared_ptr<Waypoint> WaypointFromIdentifier(WaypointIdentifier identifier) const;
    
    static WaypointPath
    FindPathInGraph(const std::shared_ptr<Waypoint> &origin_waypoint,
                    const std::shared_ptr<Waypoint> &destination_waypoint,
                    const std::function<bool(const WaypointPair &waypoint_pair, const WaypointInfoPair &info_pair,
                                             std::vector<std::shared_ptr<Waypoint>> &inserted_waypoints)> &can_search);
    
    static std::vector<WaypointPath>
    FindKPathInGraph(const std::shared_ptr<Waypoint> &origin_waypoint,
                     const std::shared_ptr<Waypoint> &destination_waypoint,
                     int k,
                     const std::function<WaypointPath (const std::shared_ptr<Waypoint> &spur_waypoint,
                                                       const std::shared_ptr<Waypoint> &destination_waypoint)> &find_path);

protected:
    std::map<WaypointIdentifier, std::shared_ptr<Waypoint>> waypoint_map_;
};
    
double CosinTurnAngle(const std::shared_ptr<Waypoint> &previous, const std::shared_ptr<Waypoint> &current, const std::shared_ptr<Waypoint> &next);
}
#endif /* AirwayGraph_h */
