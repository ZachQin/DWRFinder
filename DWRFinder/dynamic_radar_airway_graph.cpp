//
//  dynamic_radar_airway_graph.cpp
//  DWRFinder
//
//  Created by ZachQin on 2017/3/8.
//  Copyright © 2017年 Zach. All rights reserved.
//

#include "dynamic_radar_airway_graph.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
#include <iomanip>
#include "raster_graph.hpp"
#include "coordinate_convert.h"

namespace dwr {

#define RAD_TO_DEG	57.29577951308232
#define DEG_TO_RAD	.0174532925199432958

WorldFileInfo::WorldFileInfo(const char* path) {
    std::ifstream inf(path);
    if (!inf.is_open()) {
        return;
    }
    inf >> A >> B >> D >> E >> C >> F;
}

Pixel coordinateToPixel(double x, double y, const WorldFileInfo &w) {
    Pixel pixel;
    pixel.x = (w.E * x - w.B * y + w.B * w.F - w.E * w.C) / (w.A * w.E - w.D * w.B);
    pixel.y = (-w.D * x + w.A * y + w.D * w.C - w.A * w.F) / (w.A * w.E - w.D * w.B);
    return pixel;
}

GeoProj pixelToCoordinate(Pixel pixel, const WorldFileInfo &w) {
    GeoProj xy;
    xy.x = w.A * pixel.x + w.B * pixel.y + w.C;
    xy.y = w.D * pixel.x + w.E * pixel.y + w.F;
    return xy;
}

std::string lonlatToString(double lon, double lat) {
    std::ostringstream textStream;
    double lonDeg = lon * RAD_TO_DEG;
    textStream << std::fixed << std::setprecision(2) << lonDeg;
    if (lonDeg >= 0) {
        textStream << "E";
    } else {
        textStream << "W";
    }
    double latDeg = lat * RAD_TO_DEG;
    textStream << latDeg;
    if (latDeg >= 0) {
        textStream << "N";
    } else {
        textStream << "S";
    }
    return textStream.str();
}

std::shared_ptr<AirwayPoint> NodeInfoToAirwayPoint(const NodeInfo &info, const WorldFileInfo &w) {
    GeoProj xy = pixelToCoordinate(info.pixel, w);
    double lon, lat;
    MercToLonLat(xy.x, xy.y, &lon, &lat);
    auto name = lonlatToString(lon, lat);
    std::shared_ptr<AirwayPoint> userWaypoint(new AirwayPoint(kNoAirwaypointID, name, lon, lat));
    userWaypoint->userWaypoint = true;
    return userWaypoint;
}

void DynamicRadarAirwayGraph::prebuild(const WorldFileInfo &worldFileInfo) {
    worldFileInfo_ = worldFileInfo;
//    for (Vertex startVertex = 0; startVertex < adjacencyList_.size(); startVertex++) {
//        auto neighbors = adjacencyList_[startVertex];
//        for (auto &neighbor: neighbors) {
//            Vertex endVertex = neighbor.target;
//            auto startAirwayPoint = airwayPointVector_[startVertex];
//            auto endAirwayPoint = airwayPointVector_[endVertex];
//            // 开始遍历
//            Pixel startPixel = coordinateToPixel(startAirwayPoint.x, startAirwayPoint.y, worldFileInfo);
//            Pixel endPixel = coordinateToPixel(endAirwayPoint.x, endAirwayPoint.y, worldFileInfo);
//            std::vector<Pixel> linePixels;
//            BresenhamLine(startPixel, endPixel, linePixels);
//            for (auto &point: linePixels) {
//                UndirectedEdge e = UndirectedEdge(startVertex, endVertex);
//                pixelToEdgeTable_.insert(std::make_pair(point, e));
//            }
//            //结束遍历
//        }
//    }
    
    auto traverseFunction = [&](const std::shared_ptr<AirwayPoint> &startWpt, const std::shared_ptr<AirwayPoint> &endWpt, GeoDistance d) {
        // 更新坐标
        if (startWpt->coordinate.x == kNoCoordinate) {
            LonLatToMerc(startWpt->location.longitude, startWpt->location.latitude, &startWpt->coordinate.x, &startWpt->coordinate.y);
        }
        if (endWpt->coordinate.x == kNoCoordinate) {
            LonLatToMerc(endWpt->location.longitude, endWpt->location.latitude, &endWpt->coordinate.x, &endWpt->coordinate.y);
        }
        Pixel startPixel = coordinateToPixel(startWpt->coordinate.x, startWpt->coordinate.y, worldFileInfo);
        Pixel endPixel = coordinateToPixel(endWpt->coordinate.x, endWpt->coordinate.y, worldFileInfo);
        std::vector<Pixel> linePixels = BresenhamLine(startPixel, endPixel);
        for (auto &point: linePixels) {
            auto upair = UndirectedAirwayPointPair(startWpt, endWpt);
            pixelToEdgeTable_.insert(std::make_pair(point, upair));
        }
    };
    this->ForEach(traverseFunction);
}

void DynamicRadarAirwayGraph::UpdateBlock(const char *mask, int width, int height) {
    rasterGraph_.SetRasterData(mask, width, height);
    // 更新阻塞集合
    blockSet_.clear();
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            if (mask[i * width + j] > 0) {
                Pixel pixel = Pixel(j, i);
                auto edgeIter = pixelToEdgeTable_.find(pixel);
                if (edgeIter != pixelToEdgeTable_.end()) {
                    blockSet_.insert(edgeIter->second);
                }
            }
        }
    }
}

std::vector<std::shared_ptr<AirwayPoint>> DynamicRadarAirwayGraph::GetDynamicFullPath(AirwayPointID sourceIdentity, AirwayPointID destinIdentity) {
    std::map<AirwayPointPair, std::vector<NodeInfo>> userWaypointMap;
    auto canSearch = [&](const AirwayPointPair &pair) {
        if (blockSet_.find(UndirectedAirwayPointPair(pair)) == blockSet_.end()) {
            return true;
        }
        const std::shared_ptr<AirwayPoint> ap1 = pair.first;
        const std::shared_ptr<AirwayPoint> ap2 = pair.second;
        Pixel source = coordinateToPixel(ap1->coordinate.x, ap1->coordinate.y, worldFileInfo_);
        Pixel destin = coordinateToPixel(ap2->coordinate.x, ap2->coordinate.y, worldFileInfo_);
        std::vector<std::vector<Pixel>> nodes = rasterGraph_.GetNodes(source, destin, 3);
        std::vector<NodeInfo> infos = rasterGraph_.GetPath(source, destin, nodes);
        if (infos.empty()) {
            return false;
        } else {
            // 去掉首尾
            userWaypointMap[std::make_pair(ap1, ap2)] = std::vector<NodeInfo>(infos.begin() + 1, infos.end() - 1);
            return true;
        }
    };
    std::vector<std::shared_ptr<AirwayPoint>> fullPath;
    std::vector<std::shared_ptr<AirwayPoint>> partialPath = GetPath(sourceIdentity, destinIdentity, canSearch);
    if (partialPath.size() > 0) {
        fullPath.push_back(partialPath[0]);
    }
    for (int i = 1; i < partialPath.size(); i++) {
        auto start = partialPath[i - 1];
        auto end = partialPath[i];
        auto it = userWaypointMap.find(std::make_pair(start, end));
        if (it != userWaypointMap.end()) {
            for (auto &up: it->second) {
                auto ap = NodeInfoToAirwayPoint(up, worldFileInfo_);
                fullPath.push_back(ap);
            }
        }
        fullPath.push_back(end);
    }
    return fullPath;
};

//void DynamicRadarAirwayGraph::LogBlockAirpointSegment() {
//    for (auto &block: blockSet_) {
//        std::cout << "from:" << airwayPointVector_[block.small].airwayPointID << "  to:" << airwayPointVector_[block.big].airwayPointID << std::endl;
//    }
//}

}
