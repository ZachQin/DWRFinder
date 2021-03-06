//
//  AirwayGraph.cc
//  DWR
//
//  Created by ZachQin on 2017/3/3.
//  Copyright © 2017年 Zach. All rights reserved.
//

#include "airway_graph.h"

#include <queue>
#include <fstream>
#include <string>
#include <memory>
#include <utility>

namespace dwr {

AirwayGraph::AirwayGraph(const char *path) {
    this->LoadFromFile(path);
}

void AirwayGraph::AddWaypoint(WaypointIdentifier identifier,
                              const std::string &name,
                              GeoRad longitude,
                              GeoRad latitude) {
    waypoint_map_[identifier] = std::make_shared<Waypoint>(identifier, name, longitude, latitude);
}

void AirwayGraph::RemoveAirwaySegments(const WaypointPtr &waypoint) {
    // Delete self from neibors.
    for (auto &neibor : waypoint->neibors) {
        auto target = neibor.target.lock();
        target->neibors.erase(std::remove_if(target->neibors.begin(), target->neibors.end(), [&](Neighbor &neib){
            return neib.target.lock() == waypoint;
        }), target->neibors.end());
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

void AirwayGraph::AddAirwaySegment(const WaypointPtr &waypoint1,
                                   const WaypointPtr &waypoint2) {
    GeoDistance distance = Waypoint::Distance(*waypoint1, *waypoint2);
    Neighbor neibor1(waypoint2, distance);
    Neighbor neibor2(waypoint1, distance);
    if (std::find(waypoint1->neibors.begin(), waypoint1->neibors.end(), neibor1) == waypoint1->neibors.end()) {
        waypoint1->neibors.push_back(std::move(neibor1));
    }
    if (std::find(waypoint2->neibors.begin(), waypoint2->neibors.end(), neibor2) == waypoint2->neibors.end()) {
        waypoint2->neibors.push_back(std::move(neibor2));
    }
}

void AirwayGraph::AddAirwaySegment(WaypointIdentifier identifier1,
                                   WaypointIdentifier identifier2) {
    auto waypoint_iterator1 = waypoint_map_.find(identifier1);
    auto waypoint_iterator2 = waypoint_map_.find(identifier2);
    if (waypoint_iterator1 == waypoint_map_.end() ||
        waypoint_iterator2 == waypoint_map_.end()) {
        return;
    }
    AddAirwaySegment(waypoint_iterator1->second, waypoint_iterator2->second);
}

void AirwayGraph::RemoveAirwaySegment(const WaypointPtr &waypoint1,
                                      const WaypointPtr &waypoint2) {
    waypoint1->neibors.erase(std::remove_if(waypoint1->neibors.begin(), waypoint1->neibors.end(), [&](Neighbor &neib){
        return neib.target.lock() == waypoint2;
    }), waypoint1->neibors.end());
    waypoint2->neibors.erase(std::remove_if(waypoint2->neibors.begin(), waypoint2->neibors.end(), [&](Neighbor &neib){
        return neib.target.lock() == waypoint1;
    }), waypoint2->neibors.end());
}

void AirwayGraph::RemoveAirwaySegment(WaypointIdentifier identifier1,
                                      WaypointIdentifier identifier2) {
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
                                               const WaypointInfoPair &,
                                               std::vector<WaypointPtr> &)> &can_search) const {
    auto origin_iterator = waypoint_map_.find(origin_identifier);
    auto destination_iterator = waypoint_map_.find(destination_identifier);
    if (origin_iterator == waypoint_map_.end() || destination_iterator == waypoint_map_.end()) {
        return WaypointPath();
    }
    auto origin_waypoint = origin_iterator->second;
    auto destination_waypoint = destination_iterator->second;
    return FindPathInGraph(origin_waypoint, destination_waypoint, can_search);
}

std::vector<WaypointPath>
AirwayGraph::FindKPath(WaypointIdentifier origin_identifier,
                       WaypointIdentifier destination_identifier,
                       int k,
                       const std::function<WaypointPath(const ConstWaypointPtr &,
                                                        const ConstWaypointPtr &,
                                                        const std::set<WaypointPair> &)> &find_path) const {
    auto origin_iterator = waypoint_map_.find(origin_identifier);
    auto destination_iterator = waypoint_map_.find(destination_identifier);
    if (origin_iterator == waypoint_map_.end() || destination_iterator == waypoint_map_.end()) {
        return std::vector<WaypointPath>();
    }
    auto origin_waypoint = origin_iterator->second;
    auto destination_waypoint = destination_iterator->second;
    return FindKPathInGraph(origin_waypoint, destination_waypoint, k, find_path);
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
        uint32_t identifier = static_cast<uint32_t>(waypoint.identifier);
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
        uint32_t identifier = static_cast<uint32_t>(waypoint.identifier);
        of.write(reinterpret_cast<char *>(&identifier), sizeof(identifier));
        // 序列化邻接个数
        uint32_t neibor_size = static_cast<uint32_t>(waypoint.neibors.size());
        of.write(reinterpret_cast<char *>(&neibor_size), sizeof(neibor_size));
        for (auto &neibor : waypoint.neibors) {
            // 序列化相邻航路点ID
            uint32_t neibor_identifier = static_cast<uint32_t>(neibor.target.lock()->identifier);
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
        auto waypoint = std::make_shared<Waypoint>();
        // 反序列化ID
        uint32_t identifier = 0;
        inf.read(reinterpret_cast<char *>(&identifier), sizeof(identifier));
        waypoint->identifier = static_cast<WaypointIdentifier>(identifier);
        // 反序列化名称
        uint32_t name_size = 0;
        inf.read(reinterpret_cast<char *>(&name_size), sizeof(name_size));
        char *name_buff = new char[name_size + 1];
        name_buff[name_size] = 0;
        inf.read(name_buff, name_size);
        waypoint->name = std::string(name_buff);
        delete [] name_buff;
        // 反序列化经度
        double longitude = 0.0;
        inf.read(reinterpret_cast<char *>(&longitude), sizeof(longitude));
        waypoint->location.longitude = static_cast<double>(longitude);
        // 反序列化纬度
        double latitude = 0.0;
        inf.read(reinterpret_cast<char *>(&latitude), sizeof(latitude));
        waypoint->location.latitude = static_cast<double>(latitude);
        waypoint_map_[identifier] = waypoint;
    }
    for (int i = 0; i < n; i++) {
        // 反序列化ID
        uint32_t identifier = 0;
        inf.read(reinterpret_cast<char *>(&identifier), sizeof(identifier));
        WaypointPtr waypoint = waypoint_map_[static_cast<WaypointIdentifier>(identifier)];
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
            waypoint->neibors.push_back(neibor);
        }
    }
    return true;
}

void AirwayGraph::ForEach(const std::function<void(const WaypointPtr &,
                                                   const WaypointPtr &,
                                                   GeoDistance)> &traverse_function) {
    for (auto &pair : waypoint_map_) {
        auto waypoint = pair.second;
        for (auto &neibor : waypoint->neibors) {
            traverse_function(waypoint, neibor.target.lock(), neibor.distance);
        }
    }
}

std::vector<WaypointIdentifier> AirwayGraph::AllWaypointIdentifiers() const {
    std::vector<WaypointIdentifier> all_waypoint_identifier_vector;
    for (auto &pair : waypoint_map_) {
        all_waypoint_identifier_vector.push_back(pair.first);
    }
    return all_waypoint_identifier_vector;
}

WaypointPtr AirwayGraph::WaypointFromIdentifier(WaypointIdentifier identifier) const {
    auto iterator = waypoint_map_.find(identifier);
    if (iterator != waypoint_map_.end()) {
        return iterator->second;
    } else {
        return nullptr;
    }
}

static GeoDistance HeuristicDistance(const ConstWaypointPtr &waypoint1,
                                     const ConstWaypointPtr &waypoint2) {
    return Waypoint::Distance(*waypoint1, *waypoint2) * 0.9;
}

WaypointPath
AirwayGraph::FindPathInGraph(const ConstWaypointPtr &origin_waypoint,
                             const ConstWaypointPtr &destination_waypoint,
                             const std::function<bool(const WaypointPair &,
                                                      const WaypointInfoPair &,
                                                      std::vector<WaypointPtr> &)> &can_search) {
    WaypointPath result;
    std::map<ConstWaypointPtr, WaypointInfo> waypoint_info_map;
    // Init priority queue.
    auto waypoint_compare = [&waypoint_info_map](const ConstWaypointPtr &waypoint1,
                                                 const ConstWaypointPtr &waypoint2) {
        auto &waypoint_info1 = waypoint_info_map[waypoint1];
        auto &waypoint_info2 = waypoint_info_map[waypoint2];
        return waypoint_info1.estimated_distance > waypoint_info2.estimated_distance;
    };
    std::priority_queue<ConstWaypointPtr, std::vector<ConstWaypointPtr>,
                        decltype(waypoint_compare)> waypoint_queue(waypoint_compare);
    auto &origin_info = waypoint_info_map[origin_waypoint];
    origin_info.actual_distance = 0;
    origin_info.estimated_distance = HeuristicDistance(origin_waypoint, destination_waypoint);
    waypoint_queue.push(origin_waypoint);
    while (!waypoint_queue.empty()) {
        ConstWaypointPtr current_waypoint = waypoint_queue.top();
        WaypointInfo &current_info = waypoint_info_map[current_waypoint];
        waypoint_queue.pop();
        if (current_waypoint == destination_waypoint) {
            break;
        }
        for (auto &neibor : current_waypoint->neibors) {
            WaypointPtr neibor_waypoint = neibor.target.lock();
            WaypointInfo &neibor_info = waypoint_info_map[neibor_waypoint];
            std::vector<WaypointPtr> inserted_waypoints;
            if (!can_search(std::make_pair(current_waypoint, neibor_waypoint),
                            std::make_pair(current_info, neibor_info),
                            inserted_waypoints)) {
                continue;
            }
            GeoDistance distance_through_current = current_info.actual_distance;
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
            if (distance_through_current < neibor_info.actual_distance) {
                neibor_info.actual_distance = distance_through_current;
                if (inserted_waypoints.size() == 0) {
                    neibor_info.previous = current_waypoint;
                } else {
                    ConstWaypointPtr current_inserted_waypoint = neibor_waypoint;
                    for (auto iterator = inserted_waypoints.rbegin(); iterator != inserted_waypoints.rend(); iterator++) {
                        auto &current_inserted_info = waypoint_info_map[current_inserted_waypoint];
                        current_inserted_info.previous = *iterator;
                        current_inserted_waypoint = current_inserted_info.previous.lock();
                    }
                    waypoint_info_map[current_inserted_waypoint].previous = current_waypoint;
                }
                neibor_info.estimated_distance = neibor_info.actual_distance + HeuristicDistance(neibor_waypoint, destination_waypoint);
                waypoint_queue.push(neibor_waypoint);
            }
        }
    }
    auto current_waypoint = destination_waypoint;
    auto &current_info = waypoint_info_map[current_waypoint];
    if (current_info.previous.lock() == nullptr) {
        return result;
    }
    while (current_waypoint != nullptr) {
        current_info = waypoint_info_map[current_waypoint];
        result.waypoints.push_back(current_waypoint);
        result.lengths.push_back(current_info.actual_distance);
        current_waypoint = current_info.previous.lock();
    }
    std::reverse(result.waypoints.begin(), result.waypoints.end());
    std::reverse(result.lengths.begin(), result.lengths.end());
    return result;
}

std::vector<WaypointPath>
AirwayGraph::FindKPathInGraph(const ConstWaypointPtr &origin_waypoint,
                 const ConstWaypointPtr &destination_waypoint,
                 int k,
                 const std::function<WaypointPath(const ConstWaypointPtr &,
                                                  const ConstWaypointPtr &,
                                                  const std::set<WaypointPair> &)> &find_path) {
    std::vector<WaypointPath> result;
    auto path_compare = [](const WaypointPath &path1, const WaypointPath &path2) {
        return path1.lengths.back() > path2.lengths.back();
    };
    std::priority_queue<WaypointPath,
                        std::vector<WaypointPath>,
                        decltype(path_compare)> path_queue(path_compare);
    WaypointPath init_path = find_path(origin_waypoint,
                                       destination_waypoint,
                                       std::set<WaypointPair>());
    if (init_path.waypoints.size() == 0) {
        return result;
    }
    result.push_back(std::move(init_path));
    for (int kk = 1; kk < k; kk++) {
        for (int i = 0; i < result[kk - 1].GetSize() - 1; i++) {
            std::set<WaypointPair> removed_edges;
            auto spur_waypoint = result[kk - 1].waypoints[i];
            WaypointPath root_path = WaypointPath(result[kk - 1], 0, i + 1);
            for (auto &path : result) {
                if (std::equal(root_path.waypoints.begin(), root_path.waypoints.end(), path.waypoints.begin())) {
                    removed_edges.insert(std::make_pair(path.waypoints[i], path.waypoints[i + 1]));
                }
            }
            for (auto &root_path_node : root_path.waypoints) {
                if (root_path_node != spur_waypoint) {
                    for (auto &neibor : root_path_node->neibors) {
                        removed_edges.insert(std::make_pair(root_path_node, neibor.target.lock()));
                    }
                }
            }
            auto spur_path = find_path(spur_waypoint, destination_waypoint, removed_edges);
            if (spur_path.GetSize() > 0) {
                WaypointPath total_path = root_path + spur_path;
                path_queue.push(std::move(total_path));
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

}  // namespace dwr
