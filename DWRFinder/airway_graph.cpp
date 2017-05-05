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

void AirwayGraph::AddAirwayPoint(AirwayPointID identity, std::string name, double lon, double lat) {
    std::shared_ptr<AirwayPoint> point(new AirwayPoint(identity, name, lon, lat));
    airwayPointMap_.insert(std::make_pair(identity, point));
}

void AirwayGraph::AddAirwaySegment(AirwayPointID identity1, AirwayPointID identity2) {
    auto wpt1 = airwayPointMap_[identity1];
    auto wpt2 = airwayPointMap_[identity2];
    GeoDistance d = AirwayPoint::Distance(*wpt1, *wpt2);
    Neighbor neib1(wpt2, d);
    Neighbor neib2(wpt1, d);
    wpt1->neibors.push_back(neib1);
    wpt2->neibors.push_back(neib2);
}
    
/**
 使用A*算法获取路径

 @param sourceIdentity 源航路点ID
 @param destinIdentity 目的航路点ID
 @param canSearch 可达性判断
 @return 航路点路径
 */
std::vector<std::shared_ptr<AirwayPoint>> AirwayGraph::GetPath(AirwayPointID sourceIdentity, AirwayPointID destinIdentity, const std::function<bool(const AirwayPointPair &)> canSearch) {
    std::vector<std::shared_ptr<AirwayPoint>> result;
    auto sourceIt = airwayPointMap_.find(sourceIdentity);
    auto destinIt = airwayPointMap_.find(destinIdentity);
    if (sourceIt == airwayPointMap_.end() || destinIt == airwayPointMap_.end()) {
        return result;
    }
    auto sourceWpt = sourceIt->second;
    auto destinWpt = destinIt->second;
    // 初始化Cache，主要用于清除所有AirwayPoint的actualDistance和heuristicDistance
    for (auto &p : airwayPointMap_) {
        p.second->ResetCache();
    }
    // 初始化优先队列
    auto wptComp = [](const std::shared_ptr<AirwayPoint> &wpt1, const std::shared_ptr<AirwayPoint> &wpt2) {
        return wpt1->actualDistance + wpt1->heuristicDistance > wpt2->actualDistance + wpt2->heuristicDistance;
    };
    std::priority_queue<std::shared_ptr<AirwayPoint>, std::vector<std::shared_ptr<AirwayPoint>>, decltype(wptComp)> wptQueue(wptComp);
    sourceWpt->actualDistance = 0;
    sourceWpt->heuristicDistance = AirwayPoint::Distance(*sourceWpt, *destinWpt);
    wptQueue.push(sourceWpt);
    while (!wptQueue.empty()) {
        std::shared_ptr<AirwayPoint> currentWpt = wptQueue.top();
        wptQueue.pop();
        if (currentWpt == destinWpt) {
            break;
        }
        for (auto &neibor : currentWpt->neibors) {
            std::shared_ptr<AirwayPoint> neiborWpt = neibor.target.lock();
            if (!canSearch(std::make_pair(currentWpt, neiborWpt))) {
                continue;
            }
            GeoDistance distanceThroughCurrent = currentWpt->actualDistance + neibor.distance;
            if (distanceThroughCurrent < neiborWpt->actualDistance) {
                neiborWpt->actualDistance = distanceThroughCurrent;
                neiborWpt->previous = currentWpt;
                neiborWpt->heuristicDistance = AirwayPoint::Distance(*neiborWpt, *destinWpt);
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
    uint32_t n = static_cast<uint32_t>(airwayPointMap_.size());
    of.write(reinterpret_cast<char *>(&n), sizeof(n));
    // 序列化航路点基本信息
    for (auto &airwayPointPair : airwayPointMap_) {
        const AirwayPoint &wpt = *airwayPointPair.second;
        // 序列化ID
        uint32_t identity = static_cast<uint32_t>(wpt.airwayPointID);
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
    for (auto &airwayPointPair : airwayPointMap_) {
        const AirwayPoint &wpt = *airwayPointPair.second;
        // 序列化ID
        uint32_t identity = static_cast<uint32_t>(wpt.airwayPointID);
        of.write(reinterpret_cast<char *>(&identity), sizeof(identity));
        // 序列化邻接个数
        uint32_t neiborSize = static_cast<uint32_t>(wpt.neibors.size());
        of.write(reinterpret_cast<char *>(&neiborSize), sizeof(neiborSize));
        for (auto &neibor : wpt.neibors) {
            // 序列化相邻航路点ID
            uint32_t neiborID = static_cast<uint32_t>(neibor.target.lock()->airwayPointID);
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
        std::shared_ptr<AirwayPoint> wpt(new AirwayPoint);
        // 反序列化ID
        uint32_t identity = 0;
        inf.read(reinterpret_cast<char *>(&identity), sizeof(identity));
        wpt->airwayPointID = static_cast<AirwayPointID>(identity);
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
        airwayPointMap_.insert(std::make_pair(identity, wpt));
    }
    for (int i = 0; i < n; i++) {
        // 反序列化ID
        uint32_t identity = 0;
        inf.read(reinterpret_cast<char *>(&identity), sizeof(identity));
        std::shared_ptr<AirwayPoint> wpt = airwayPointMap_[static_cast<AirwayPointID>(identity)];
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
            
            Neighbor neibor(airwayPointMap_[neiborID], static_cast<GeoDistance>(distance));
            wpt->neibors.push_back(neibor);
        }
    }
    return true;
}

void AirwayGraph::ForEach(std::function<void (const std::shared_ptr<AirwayPoint> &, const std::shared_ptr<AirwayPoint> &, GeoDistance)> traverseFunction) {
    for (auto &pair : airwayPointMap_) {
        auto wpt = pair.second;
        for (auto &neibor : wpt->neibors) {
            traverseFunction(wpt, neibor.target.lock(), neibor.distance);
        }
    }
}

std::shared_ptr<AirwayPoint> AirwayGraph:: AirwayPointFromID(AirwayPointID identity) {
    auto iterator = airwayPointMap_.find(identity);
    if (iterator != airwayPointMap_.end()) {
        return iterator->second;
    } else {
        return nullptr;
    }
};
    
}
