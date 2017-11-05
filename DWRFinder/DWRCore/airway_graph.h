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

class AirwayGraph;

template <class T>
std::vector<WaypointPath>
FindKPathInGraph(T &graph,
          WaypointIdentifier origin_identifier,
          WaypointIdentifier destination_identifier,
          int k,
                 const std::function<WaypointPath (const T &temp_graph, WaypointIdentifier spur_node)> &find_path) {
    std::vector<WaypointPath> result;
    auto path_compare = [](WaypointPath &path1, WaypointPath &path2) {
        return path1.back()->actual_distance > path2.back()->actual_distance;
    };
    std::priority_queue<WaypointPath, std::vector<WaypointPath>, decltype(path_compare)> path_queue(path_compare);
    auto init_path = find_path(graph, origin_identifier);
    if (init_path.size() == 0) {
        return result;
    }
    result.push_back(std::move(init_path));
    for (int i = 1; i <= k; i++) {
        for (int j = 0; j < result[i - 1].size(); j++) {
            std::vector<WaypointPair> removed_edges;
            auto spur_node = result[i - 1][j];
            auto root_path = WaypointPath(result[i - 1].begin(), result[i - 1].begin() + j + 1);
            for (auto &path : result) {
                if (std::equal(root_path.begin(), root_path.end(), path.begin())) {
                    removed_edges.push_back(std::make_pair(path[j], path[j + 1]));
                    graph.RemoveAirwaySegment(path[j], path[j + 1]);
                }
            }
            for (auto &root_path_node : root_path) {
                if (root_path_node != spur_node) {
                    for (auto &neibor : root_path_node->neibors) {
                        removed_edges.push_back(std::make_pair(root_path_node, neibor.target.lock()));
                    }
                    graph.RemoveAirwaySegments(root_path_node);
                }
            }
            auto spur_path = find_path(graph, spur_node->waypoint_identifier);
            if (spur_path.size() > 0) {
                WaypointPath total_path;
                total_path.reserve(root_path.size() + spur_path.size() - 1);
                total_path.insert(total_path.end(), root_path.begin(), root_path.end() - 1);
                total_path.insert(total_path.end(), spur_path.begin(), spur_path.end());
                path_queue.push(std::move(total_path));
            }
            // Reversely restore the removed connection
            for (auto it = removed_edges.rbegin(); it != removed_edges.rend(); it++) {
                graph.AddAirwaySegment(it->first, it->second);
            }
        }
        if (path_queue.empty()) {
            break;
        }
        result.push_back(std::move(path_queue.top()));
        path_queue.pop();
    }
    return result;
}
    
class AirwayGraph {
public:
    AirwayGraph() = default;
    AirwayGraph(const char *path);
    AirwayGraph(const AirwayGraph &other) {
        for (auto &pair : other.waypoint_map_) {
            auto copied_waypoint = std::shared_ptr<Waypoint>(new Waypoint(*pair.second));
            waypoint_map_.emplace(pair.first, copied_waypoint);
        }
    }
    
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
    void AddAirwaySegment(const std::shared_ptr<Waypoint> &waypoint1, const std::shared_ptr<Waypoint> &waypoint2);
    
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
    void RemoveAirwaySegments(const std::shared_ptr<Waypoint> &waypoint);
    /**
     Remove the connection between two waypoint.

     @param waypoint1 First waypoint pointer.
     @param waypoint2 Second waypoint pointer.
     */
    void RemoveAirwaySegment(const std::shared_ptr<Waypoint> &waypoint1, const std::shared_ptr<Waypoint> &waypoint2);
    /**
     Remove the connection between two waypoint.

     @param identifier1 First waypoint identifier.
     @param identifier2 Second waypoint identifier.
     */
    void RemoveAirwaySegment(WaypointIdentifier identifier1, WaypointIdentifier identifier2);
    
    WaypointPath
    FindPath(const std::shared_ptr<Waypoint> &origin_waypoint,
             const std::shared_ptr<Waypoint> &destination_waypoint,
             const std::function<bool(const WaypointPair &pair,
                                      WaypointPath &inserted_waypoints)> &can_search
             = [](const WaypointPair &pair,
                  WaypointPath &inserted_waypoints) {return true;}
             ) const;
    
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
             const std::function<bool(const WaypointPair &pair,
                         WaypointPath &inserted_waypoints)> &can_search
             = [](const WaypointPair &pair,
                  WaypointPath &inserted_waypoints) {return true;}
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
              const std::function<WaypointPath (const AirwayGraph &temp_graph, WaypointIdentifier spur_node)> &find_path
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

protected:
    std::map<WaypointIdentifier, std::shared_ptr<Waypoint>> waypoint_map_;
};
    
double CosinTurnAngle(const std::shared_ptr<Waypoint> &previous, const std::shared_ptr<Waypoint> &current, const std::shared_ptr<Waypoint> &next);
std::string PathDescription(const WaypointPath &path);
}
#endif /* AirwayGraph_h */
