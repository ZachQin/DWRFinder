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

void AirwayGraph::AddWaypoint(WaypointID identity, std::string name, double lon, double lat) {
    std::shared_ptr<Waypoint> point(new Waypoint(identity, name, lon, lat));
    waypointMap_.insert(std::make_pair(identity, point));
}


void AirwayGraph::AddAirwaySegment(WaypointID identity1, WaypointID identity2) {
    auto wpt1 = waypointMap_[identity1];
    auto wpt2 = waypointMap_[identity2];
    GeoDistance d = Waypoint::Distance(*wpt1, *wpt2);
    Neighbor neib1(wpt2, d);
    Neighbor neib2(wpt1, d);
    wpt1->neibors.push_back(neib1);
    wpt2->neibors.push_back(neib2);
}

std::vector<std::shared_ptr<Waypoint>> AirwayGraph::GetPath(WaypointID originIdentity, WaypointID destinIdentity, const std::function<bool(const WaypointPair &)> canSearch) {
    std::vector<std::shared_ptr<Waypoint>> result;
    auto originIt = waypointMap_.find(originIdentity);
    auto destinIt = waypointMap_.find(destinIdentity);
    if (originIt == waypointMap_.end() || destinIt == waypointMap_.end()) {
        return result;
    }
    auto originWpt = originIt->second;
    auto destinWpt = destinIt->second;
    // Init Cache for waypoint.actualDistance and waypoint.heuristicDistance.
    for (auto &p : waypointMap_) {
        p.second->ResetCache();
    }
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
            if (!canSearch(std::make_pair(currentWpt, neiborWpt))) {
                continue;
            }
            GeoDistance distanceThroughCurrent = currentWpt->actualDistance + neibor.distance;
            if (distanceThroughCurrent < neiborWpt->actualDistance) {
                neiborWpt->actualDistance = distanceThroughCurrent;
                neiborWpt->previous = currentWpt;
                neiborWpt->heuristicDistance = Waypoint::Distance(*neiborWpt, *destinWpt);
                wptQueue.push(neiborWpt);
            }
        }
    }
    auto curWpt = destinWpt;
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
        uint32_t identity = static_cast<uint32_t>(wpt.waypointID);
        of.write(reinterpret_cast<char *>(&identity), sizeof(identity));
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
        uint32_t identity = static_cast<uint32_t>(wpt.waypointID);
        of.write(reinterpret_cast<char *>(&identity), sizeof(identity));
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
        uint32_t identity = 0;
        inf.read(reinterpret_cast<char *>(&identity), sizeof(identity));
        wpt->waypointID = static_cast<WaypointID>(identity);
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
        waypointMap_.insert(std::make_pair(identity, wpt));
    }
    for (int i = 0; i < n; i++) {
        // 反序列化ID
        uint32_t identity = 0;
        inf.read(reinterpret_cast<char *>(&identity), sizeof(identity));
        std::shared_ptr<Waypoint> wpt = waypointMap_[static_cast<WaypointID>(identity)];
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
            wpt->neibors.push_back(neibor);
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

std::shared_ptr<Waypoint> AirwayGraph:: WaypointFromID(WaypointID identity) {
    auto iterator = waypointMap_.find(identity);
    if (iterator != waypointMap_.end()) {
        return iterator->second;
    } else {
        return nullptr;
    }
};
    
}
