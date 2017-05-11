//
//  AirwayGraph.cpp
//  DWR
//
//  Created by ZachQin on 2017/3/3.
//  Copyright © 2017年 Zach. All rights reserved.
//

#include <string>
#include <queue>
#include <fstream>
#include "airway_graph.hpp"

namespace dwr {
    
AirwayGraph::AirwayGraph(const char *path) {
    this->LoadFromFile(path);
}

void AirwayGraph::AddWaypoint(WaypointID identifier, std::string name, GeoRad lon, GeoRad lat) {
    std::shared_ptr<Waypoint> point(new Waypoint(identifier, name, lon, lat));
    waypointMap_.insert(std::make_pair(identifier, point));
}
    
void AirwayGraph::RemoveWaypoint(WaypointID identifier) {
    auto wptIt = waypointMap_.find(identifier);
    if (wptIt == waypointMap_.end()) {
        return;
    }
    auto wpt = wptIt->second;
    for (auto &neibor : wpt->neibors) {
        auto target = neibor.target.lock();
        Neighbor deletedNeibor(wpt, neibor.distance);
        target->neibors.erase(deletedNeibor);
    }
    waypointMap_.erase(identifier);
}

void AirwayGraph::AddAirwaySegment(WaypointID identifier1, WaypointID identifier2) {
    auto wpt1It = waypointMap_.find(identifier1);
    auto wpt2It = waypointMap_.find(identifier2);
    if (wpt1It == waypointMap_.end() || wpt2It == waypointMap_.end()) {
        return;
    }
    GeoDistance d = Waypoint::Distance(*wpt1It->second, *wpt2It->second);
    Neighbor neib1(wpt2It->second, d);
    Neighbor neib2(wpt1It->second, d);
    wpt1It->second->neibors.insert(neib1);
    wpt2It->second->neibors.insert(neib2);
}

std::vector<std::shared_ptr<Waypoint>> AirwayGraph::GetPath(WaypointID originIdentifier, WaypointID destinIdentifier, const std::function<bool(const WaypointPair &, std::vector<std::shared_ptr<Waypoint>> &)> canSearch) {
    std::vector<std::shared_ptr<Waypoint>> result;
    auto originIt = waypointMap_.find(originIdentifier);
    auto destinIt = waypointMap_.find(destinIdentifier);
    if (originIt == waypointMap_.end() || destinIt == waypointMap_.end()) {
        return result;
    }
    auto originWpt = originIt->second;
    auto destinWpt = destinIt->second;
    // Init Cache for waypoint.actualDistance and waypoint.heuristicDistance.
    for (auto &p : waypointMap_) {
        p.second->ResetCache();
    }
    std::vector<std::shared_ptr<Waypoint>> insertedWaypointVector;
    // Init priority queue.
    auto wptComp = [](const std::shared_ptr<Waypoint> &wpt1, const std::shared_ptr<Waypoint> &wpt2) {
        return wpt1->actualDistance + wpt1->heuristicDistance > wpt2->actualDistance + wpt2->heuristicDistance;
    };
    std::priority_queue<std::shared_ptr<Waypoint>, std::vector<std::shared_ptr<Waypoint>>, decltype(wptComp)> wptQueue(wptComp);
    originWpt->actualDistance = 0;
    originWpt->heuristicDistance = Waypoint::Distance(*originWpt, *destinWpt);
    wptQueue.push(originWpt);
    while (!wptQueue.empty()) {
        std::shared_ptr<Waypoint> currentWpt = wptQueue.top();
        wptQueue.pop();
        if (currentWpt == destinWpt) {
            break;
        }
        for (auto &neibor : currentWpt->neibors) {
            std::shared_ptr<Waypoint> neiborWpt = neibor.target.lock();
            std::vector<std::shared_ptr<Waypoint>> insertedWaypoints;
            if (!canSearch(std::make_pair(currentWpt, neiborWpt), insertedWaypoints)) {
                continue;
            }
            // Store the strong pointer to prevent the inserted waypoints released.
            for (auto &insertedWaypoint : insertedWaypoints) {
                insertedWaypointVector.push_back(insertedWaypoint);
            }
            GeoDistance distanceThroughCurrent = currentWpt->actualDistance;
            if (insertedWaypoints.size() > 0) {
                for (int i = 0; i < insertedWaypoints.size(); i++) {
                    if (i == 0) {
                        distanceThroughCurrent += Waypoint::Distance(*currentWpt, *insertedWaypoints[i]);
                    } else {
                        distanceThroughCurrent += Waypoint::Distance(*insertedWaypoints[i - 1], *insertedWaypoints[i]);
                    }
                }
                distanceThroughCurrent += Waypoint::Distance(*insertedWaypoints.back(), *neiborWpt);
            } else {
                distanceThroughCurrent += neibor.distance;
            }
            if (distanceThroughCurrent < neiborWpt->actualDistance) {
                neiborWpt->actualDistance = distanceThroughCurrent;
                if (insertedWaypoints.size() == 0) {
                    neiborWpt->previous = currentWpt;
                } else {
                    std::shared_ptr<Waypoint> curWpt = neiborWpt;
                    for (auto it = insertedWaypoints.rbegin(); it != insertedWaypoints.rend(); it++) {
                        curWpt->previous = *it;
                        curWpt = curWpt->previous.lock();
                    }
                    curWpt->previous = currentWpt;
                }
                neiborWpt->heuristicDistance = Waypoint::Distance(*neiborWpt, *destinWpt);
                wptQueue.push(neiborWpt);
            }
        }
    }
    auto curWpt = destinWpt;
    if (curWpt->previous.lock() == nullptr) {
        return result;
    }
    while (curWpt != nullptr) {
        result.push_back(curWpt);
        curWpt = curWpt->previous.lock();
    }
    std::reverse(result.begin(), result.end());
    return result;
}

bool AirwayGraph::SaveToFile(const std::string &path) const {
    std::ofstream of(path, std::ios::binary);
    if (!of.is_open()) {
        return false;
    }
    // 序列化航路点数量
    uint32_t n = static_cast<uint32_t>(waypointMap_.size());
    of.write(reinterpret_cast<char *>(&n), sizeof(n));
    // 序列化航路点基本信息
    for (auto &waypointPair : waypointMap_) {
        const Waypoint &wpt = *waypointPair.second;
        // 序列化ID
        uint32_t identifier = static_cast<uint32_t>(wpt.waypointID);
        of.write(reinterpret_cast<char *>(&identifier), sizeof(identifier));
        // 序列化名称
        uint32_t nameSize = static_cast<uint32_t>(wpt.name.size());
        of.write(reinterpret_cast<char *>(&nameSize), sizeof(nameSize));
        of.write(wpt.name.c_str(), nameSize);
        // 序列化经度
        double longitude = static_cast<double>(wpt.location.longitude);
        of.write(reinterpret_cast<char *>(&longitude), sizeof(longitude));
        // 序列化纬度
        double latitude = static_cast<double>(wpt.location.latitude);
        of.write(reinterpret_cast<char *>(&latitude), sizeof(latitude));
    }
    // 序列化航路点邻接信息
    for (auto &waypointPair : waypointMap_) {
        const Waypoint &wpt = *waypointPair.second;
        // 序列化ID
        uint32_t identifier = static_cast<uint32_t>(wpt.waypointID);
        of.write(reinterpret_cast<char *>(&identifier), sizeof(identifier));
        // 序列化邻接个数
        uint32_t neiborSize = static_cast<uint32_t>(wpt.neibors.size());
        of.write(reinterpret_cast<char *>(&neiborSize), sizeof(neiborSize));
        for (auto &neibor : wpt.neibors) {
            // 序列化相邻航路点ID
            uint32_t neiborID = static_cast<uint32_t>(neibor.target.lock()->waypointID);
            of.write(reinterpret_cast<char *>(&neiborID), sizeof(neiborID));
            // 序列化相邻航路点距离
            double neiborDistance = static_cast<double>(neibor.distance);
            of.write(reinterpret_cast<char *>(&neiborDistance), sizeof(neiborDistance));
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
        std::shared_ptr<Waypoint> wpt(new Waypoint);
        // 反序列化ID
        uint32_t identifier = 0;
        inf.read(reinterpret_cast<char *>(&identifier), sizeof(identifier));
        wpt->waypointID = static_cast<WaypointID>(identifier);
        // 反序列化名称
        uint32_t nameSize = 0;
        inf.read(reinterpret_cast<char *>(&nameSize), sizeof(nameSize));
        char nameBuff[nameSize + 1];
        nameBuff[nameSize] = 0;
        inf.read(nameBuff, nameSize);
        wpt->name = std::string(nameBuff);
        // 反序列化经度
        double longitude = 0.0;
        inf.read(reinterpret_cast<char *>(&longitude), sizeof(longitude));
        wpt->location.longitude = static_cast<double>(longitude);
        // 反序列化纬度
        double latitude = 0.0;
        inf.read(reinterpret_cast<char *>(&latitude), sizeof(latitude));
        wpt->location.latitude = static_cast<double>(latitude);
        waypointMap_.insert(std::make_pair(identifier, wpt));
    }
    for (int i = 0; i < n; i++) {
        // 反序列化ID
        uint32_t identifier = 0;
        inf.read(reinterpret_cast<char *>(&identifier), sizeof(identifier));
        std::shared_ptr<Waypoint> wpt = waypointMap_[static_cast<WaypointID>(identifier)];
        // 反序列化邻接个数
        uint32_t neiborSize = 0;
        inf.read(reinterpret_cast<char *>(&neiborSize), sizeof(neiborSize));
        for (int j = 0; j < neiborSize; j++) {
            // 反序列化相邻ID
            uint32_t neiborID = 0;
            inf.read(reinterpret_cast<char *>(&neiborID), sizeof(neiborID));
            // 反序列化相邻距离
            double distance = 0.0;
            inf.read(reinterpret_cast<char *>(&distance), sizeof(distance));
            
            Neighbor neibor(waypointMap_[neiborID], static_cast<GeoDistance>(distance));
            wpt->neibors.insert(neibor);
        }
    }
    return true;
}

void AirwayGraph::ForEach(std::function<void (const std::shared_ptr<Waypoint> &, const std::shared_ptr<Waypoint> &, GeoDistance)> traverseFunction) {
    for (auto &pair : waypointMap_) {
        auto wpt = pair.second;
        for (auto &neibor : wpt->neibors) {
            traverseFunction(wpt, neibor.target.lock(), neibor.distance);
        }
    }
}

std::shared_ptr<Waypoint> AirwayGraph:: WaypointFromID(WaypointID identifier) {
    auto iterator = waypointMap_.find(identifier);
    if (iterator != waypointMap_.end()) {
        return iterator->second;
    } else {
        return nullptr;
    }
};
    
}
