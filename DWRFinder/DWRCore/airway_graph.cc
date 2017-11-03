//
//  AirwayGraph.cc
//  DWR
//
//  Created by ZachQin on 2017/3/3.
//  Copyright © 2017年 Zach. All rights reserved.
//

#include "airway_graph.h"

#include <assert.h>
#include <queue>
#include <fstream>

namespace dwr {
    
AirwayGraph::AirwayGraph(const char *path) {
    this->LoadFromFile(path);
}

void AirwayGraph::AddWaypoint(WaypointIdentifier identifier, const std::string &name, GeoRad longitude, GeoRad latitude) {
    std::shared_ptr<Waypoint> point(new Waypoint(identifier, name, longitude, latitude));
    waypoint_map_.emplace(identifier, point);
}
    
void AirwayGraph::RemoveAirwaySegments(const std::shared_ptr<Waypoint> &waypoint) {
    // Delete self from neibors.
    for (auto &neibor : waypoint->neibors) {
        auto target = neibor.target.lock();
        Neighbor deleted_neibor(waypoint, neibor.distance);
        target->neibors.erase(deleted_neibor);
    }
    waypoint->neibors.clear();
}
    
void AirwayGraph::RemoveWaypoint(WaypointIdentifier identifier) {
    auto waypoint_iterator = waypoint_map_.find(identifier);
    if (waypoint_iterator == waypoint_map_.end()) {
        return;
    }
    RemoveAirwaySegments(waypoint_iterator->second);
    waypoint_map_.erase(identifier);
}

void AirwayGraph::AddAirwaySegment(const std::shared_ptr<Waypoint> &waypoint1, const std::shared_ptr<Waypoint> &waypoint2) {
    GeoDistance distance = Waypoint::Distance(*waypoint1, *waypoint2);
    Neighbor neibor1(waypoint2, distance);
    Neighbor neibor2(waypoint1, distance);
    waypoint1->neibors.insert(std::move(neibor1));
    waypoint2->neibors.insert(std::move(neibor2));
}
    
void AirwayGraph::AddAirwaySegment(WaypointIdentifier identifier1, WaypointIdentifier identifier2) {
    auto waypoint_iterator1 = waypoint_map_.find(identifier1);
    auto waypoint_iterator2 = waypoint_map_.find(identifier2);
    if (waypoint_iterator1 == waypoint_map_.end() || waypoint_iterator2 == waypoint_map_.end()) {
        return;
    }
    AddAirwaySegment(waypoint_iterator1->second, waypoint_iterator2->second);
}

void AirwayGraph::RemoveAirwaySegment(const std::shared_ptr<Waypoint> &waypoint1, const std::shared_ptr<Waypoint> &waypoint2) {
    GeoDistance distance = Waypoint::Distance(*waypoint1, *waypoint2);
    Neighbor neibor1(waypoint2, distance);
    Neighbor neibor2(waypoint1, distance);
    waypoint1->neibors.erase(neibor1);
    waypoint2->neibors.erase(neibor2);
}

void AirwayGraph::RemoveAirwaySegment(WaypointIdentifier identifier1, WaypointIdentifier identifier2) {
    auto waypoint_iterator1 = waypoint_map_.find(identifier1);
    auto waypoint_iterator2 = waypoint_map_.find(identifier2);
    if (waypoint_iterator1 == waypoint_map_.end() || waypoint_iterator2 == waypoint_map_.end()) {
        return;
    }
    RemoveAirwaySegment(waypoint_iterator1->second, waypoint_iterator2->second);
}
    
WaypointPath
AirwayGraph::FindPath(WaypointIdentifier origin_identifier,
                      WaypointIdentifier destination_identifier,
                      const std::function<bool(const WaypointPair &,
                                               WaypointPath &)> &can_search) const {
    WaypointPath result;
    auto origin_iterator = waypoint_map_.find(origin_identifier);
    auto destination_iterator = waypoint_map_.find(destination_identifier);
    if (origin_iterator == waypoint_map_.end() || destination_iterator == waypoint_map_.end()) {
        return result;
    }
    auto origin_waypoint = origin_iterator->second;
    auto destination_waypoint = destination_iterator->second;
    // Init Cache for waypoint.actual_distance and waypoint.heuristic_distance.
    for (auto &p : waypoint_map_) {
        p.second->ResetCache();
    }
    WaypointPath inserted_waypoint_vector;
    // Init priority queue.
    auto waypoint_compare = [](const std::shared_ptr<Waypoint> &waypoint1, const std::shared_ptr<Waypoint> &waypoint2) {
        return waypoint1->actual_distance + waypoint1->heuristic_distance > waypoint2->actual_distance + waypoint2->heuristic_distance;
    };
    std::priority_queue<std::shared_ptr<Waypoint>, std::vector<std::shared_ptr<Waypoint>>, decltype(waypoint_compare)> waypoint_queue(waypoint_compare);
    origin_waypoint->actual_distance = 0;
    origin_waypoint->heuristic_distance = Waypoint::Distance(*origin_waypoint, *destination_waypoint);
    waypoint_queue.push(origin_waypoint);
    while (!waypoint_queue.empty()) {
        std::shared_ptr<Waypoint> current_waypoint = waypoint_queue.top();
        waypoint_queue.pop();
        if (current_waypoint == destination_waypoint) {
            break;
        }
        for (auto &neibor : current_waypoint->neibors) {
            std::shared_ptr<Waypoint> neibor_waypoint = neibor.target.lock();
            WaypointPath inserted_waypoints;
            if (!can_search(std::make_pair(current_waypoint, neibor_waypoint), inserted_waypoints)) {
                continue;
            }
            // Store the strong pointer to prevent the inserted waypoints released.
            for (auto &inserted_waypoint : inserted_waypoints) {
                inserted_waypoint_vector.push_back(inserted_waypoint);
            }
            GeoDistance distance_through_current = current_waypoint->actual_distance;
            if (inserted_waypoints.size() > 0) {
                for (int i = 0; i < inserted_waypoints.size(); i++) {
                    if (i == 0) {
                        distance_through_current += Waypoint::Distance(*current_waypoint, *inserted_waypoints[i]);
                    } else {
                        distance_through_current += Waypoint::Distance(*inserted_waypoints[i - 1], *inserted_waypoints[i]);
                    }
                }
                distance_through_current += Waypoint::Distance(*inserted_waypoints.back(), *neibor_waypoint);
            } else {
                distance_through_current += neibor.distance;
            }
            if (distance_through_current < neibor_waypoint->actual_distance) {
                neibor_waypoint->actual_distance = distance_through_current;
                if (inserted_waypoints.size() == 0) {
                    neibor_waypoint->previous = current_waypoint;
                } else {
                    std::shared_ptr<Waypoint> current_inserted_waypoint = neibor_waypoint;
                    for (auto iterator = inserted_waypoints.rbegin(); iterator != inserted_waypoints.rend(); iterator++) {
                        current_inserted_waypoint->previous = *iterator;
                        current_inserted_waypoint = current_inserted_waypoint->previous.lock();
                    }
                    current_inserted_waypoint->previous = current_waypoint;
                }
                neibor_waypoint->heuristic_distance = Waypoint::Distance(*neibor_waypoint, *destination_waypoint);
                waypoint_queue.push(neibor_waypoint);
            }
        }
    }
    auto current_waypoint = destination_waypoint;
    if (current_waypoint->previous.lock() == nullptr) {
        return result;
    }
    while (current_waypoint != nullptr) {
        result.push_back(current_waypoint);
        current_waypoint = current_waypoint->previous.lock();
    }
    std::reverse(result.begin(), result.end());
    return result;
}

std::vector<WaypointPath>
AirwayGraph::FindKPath(WaypointIdentifier origin_identifier,
                       WaypointIdentifier destination_identifier,
                       int k,
                       const std::function<WaypointPath (const AirwayGraph &, WaypointIdentifier)> &find_path
                       ) const {
    auto copied_graph = AirwayGraph(*this);
    return FindKPathInGraph(copied_graph, origin_identifier, destination_identifier, k, find_path);
}

bool AirwayGraph::SaveToFile(const std::string &path) const {
    std::ofstream of(path, std::ios::binary);
    if (!of.is_open()) {
        return false;
    }
    // 序列化航路点数量
    uint32_t n = static_cast<uint32_t>(waypoint_map_.size());
    of.write(reinterpret_cast<char *>(&n), sizeof(n));
    // 序列化航路点基本信息
    for (auto &waypoint_pair : waypoint_map_) {
        const Waypoint &waypoint = *waypoint_pair.second;
        // 序列化ID
        uint32_t identifier = static_cast<uint32_t>(waypoint.waypoint_identifier);
        of.write(reinterpret_cast<char *>(&identifier), sizeof(identifier));
        // 序列化名称
        uint32_t name_size = static_cast<uint32_t>(waypoint.name.size());
        of.write(reinterpret_cast<char *>(&name_size), sizeof(name_size));
        of.write(waypoint.name.c_str(), name_size);
        // 序列化经度
        double longitude = static_cast<double>(waypoint.location.longitude);
        of.write(reinterpret_cast<char *>(&longitude), sizeof(longitude));
        // 序列化纬度
        double latitude = static_cast<double>(waypoint.location.latitude);
        of.write(reinterpret_cast<char *>(&latitude), sizeof(latitude));
    }
    // 序列化航路点邻接信息
    for (auto &waypoint_pair : waypoint_map_) {
        const Waypoint &waypoint = *waypoint_pair.second;
        // 序列化ID
        uint32_t identifier = static_cast<uint32_t>(waypoint.waypoint_identifier);
        of.write(reinterpret_cast<char *>(&identifier), sizeof(identifier));
        // 序列化邻接个数
        uint32_t neibor_size = static_cast<uint32_t>(waypoint.neibors.size());
        of.write(reinterpret_cast<char *>(&neibor_size), sizeof(neibor_size));
        for (auto &neibor : waypoint.neibors) {
            // 序列化相邻航路点ID
            uint32_t neibor_identifier = static_cast<uint32_t>(neibor.target.lock()->waypoint_identifier);
            of.write(reinterpret_cast<char *>(&neibor_identifier), sizeof(neibor_identifier));
            // 序列化相邻航路点距离
            double neibor_distance = static_cast<double>(neibor.distance);
            of.write(reinterpret_cast<char *>(&neibor_distance), sizeof(neibor_distance));
        }
    }
    return true;
}

bool AirwayGraph::LoadFromFile(const std::string &path) {
    std::ifstream inf(path, std::ios::binary);
    if (!inf.is_open()) {
        return false;
    }
    uint32_t n = 0;
    inf.read(reinterpret_cast<char *>(&n), sizeof(n));
    for (int i = 0; i < n; i++) {
        std::shared_ptr<Waypoint> waypoint(new Waypoint);
        // 反序列化ID
        uint32_t identifier = 0;
        inf.read(reinterpret_cast<char *>(&identifier), sizeof(identifier));
        waypoint->waypoint_identifier = static_cast<WaypointIdentifier>(identifier);
        // 反序列化名称
        uint32_t name_size = 0;
        inf.read(reinterpret_cast<char *>(&name_size), sizeof(name_size));
        char name_buff[name_size + 1];
        name_buff[name_size] = 0;
        inf.read(name_buff, name_size);
        waypoint->name = std::string(name_buff);
        // 反序列化经度
        double longitude = 0.0;
        inf.read(reinterpret_cast<char *>(&longitude), sizeof(longitude));
        waypoint->location.longitude = static_cast<double>(longitude);
        // 反序列化纬度
        double latitude = 0.0;
        inf.read(reinterpret_cast<char *>(&latitude), sizeof(latitude));
        waypoint->location.latitude = static_cast<double>(latitude);

        waypoint_map_.emplace(identifier, waypoint);
    }
    for (int i = 0; i < n; i++) {
        // 反序列化ID
        uint32_t identifier = 0;
        inf.read(reinterpret_cast<char *>(&identifier), sizeof(identifier));
        std::shared_ptr<Waypoint> waypoint = waypoint_map_[static_cast<WaypointIdentifier>(identifier)];
        // 反序列化邻接个数
        uint32_t neibor_size = 0;
        inf.read(reinterpret_cast<char *>(&neibor_size), sizeof(neibor_size));
        for (int j = 0; j < neibor_size; j++) {
            // 反序列化相邻ID
            uint32_t neibor_identifier = 0;
            inf.read(reinterpret_cast<char *>(&neibor_identifier), sizeof(neibor_identifier));
            // 反序列化相邻距离
            double distance = 0.0;
            inf.read(reinterpret_cast<char *>(&distance), sizeof(distance));
            
            Neighbor neibor(waypoint_map_[neibor_identifier], static_cast<GeoDistance>(distance));
            waypoint->neibors.insert(neibor);
        }
    }
    return true;
}

void AirwayGraph::ForEach(const std::function<void (const std::shared_ptr<Waypoint> &, const std::shared_ptr<Waypoint> &, GeoDistance)> &traverse_function) {
    for (auto &pair : waypoint_map_) {
        auto waypoint = pair.second;
        for (auto &neibor : waypoint->neibors) {
            traverse_function(waypoint, neibor.target.lock(), neibor.distance);
        }
    }
}
    
std::vector<WaypointIdentifier> AirwayGraph::AllWaypointIdentifiers() {
    std::vector<WaypointIdentifier> all_waypoint_identifier_vector;
    for (auto &pair : waypoint_map_) {
        all_waypoint_identifier_vector.push_back(pair.first);
    }
    return all_waypoint_identifier_vector;
}

std::shared_ptr<Waypoint> AirwayGraph::WaypointFromIdentifier(WaypointIdentifier identifier) {
    auto iterator = waypoint_map_.find(identifier);
    if (iterator != waypoint_map_.end()) {
        return iterator->second;
    } else {
        return nullptr;
    }
}

double CosinTurnAngle(const std::shared_ptr<Waypoint> &previous, const std::shared_ptr<Waypoint> &current, const std::shared_ptr<Waypoint> &next) {
    assert(previous->coordinate.x != kNoCoordinate);
    assert(current->coordinate.x != kNoCoordinate);
    assert(next->coordinate.x != kNoCoordinate);
    double pc_x = current->coordinate.x - previous->coordinate.x;
    double pc_y = current->coordinate.y - previous->coordinate.y;
    double cn_x = next->coordinate.x - current->coordinate.x;
    double cn_y = next->coordinate.y - current->coordinate.y;
    return (pc_x * cn_x + pc_y * cn_y) / (sqrt(pc_x * pc_x + pc_y * pc_y) * sqrt(cn_x * cn_x + cn_y * cn_y));
}

}
