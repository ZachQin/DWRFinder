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

void pixelToCoordinate(Pixel pixel, double *x, double *y, const WorldFileInfo &w) {
    *x = w.A * pixel.x + w.B * pixel.y + w.C;
    *y = w.D * pixel.x + w.E * pixel.y + w.F;
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

AirwayPoint NodeInfoToAirwayPoint(const NodeInfo &info, const WorldFileInfo &w) {
    double x, y;
    pixelToCoordinate(info.pixel, &x, &y, w);
    double lon, lat;
    MercToLonLat(x, y, &lon, &lat);
    auto name = lonlatToString(lon, lat);
    AirwayPoint userWaypoint(kNoAirwaypointID, name, x, y, lon, lat);
    return userWaypoint;
}

void DynamicRadarAirwayGraph::prebuild(const WorldFileInfo &worldFileInfo) {
    worldFileInfo_ = worldFileInfo;
    for (Vertex startVertex = 0; startVertex < adjacencyList_.size(); startVertex++) {
        auto neighbors = adjacencyList_[startVertex];
        for (auto &neighbor: neighbors) {
            Vertex endVertex = neighbor.target;
            auto startAirwayPoint = airwayPointVector_[startVertex];
            auto endAirwayPoint = airwayPointVector_[endVertex];
            // 开始遍历
            Pixel startPixel = coordinateToPixel(startAirwayPoint.x, startAirwayPoint.y, worldFileInfo);
            Pixel endPixel = coordinateToPixel(endAirwayPoint.x, endAirwayPoint.y, worldFileInfo);
            std::vector<Pixel> linePixels;
            BresenhamLine(startPixel, endPixel, linePixels);
            for (auto &point: linePixels) {
                Edge e = Edge(startVertex, endVertex);
                pixelToEdgeTable_.insert(std::make_pair(point, e));
            }
            //结束遍历
        }
    }
}

void DynamicRadarAirwayGraph::UpdateBlock(const char *mask, int width, int height) {
    radarMask_ = mask;
    radarWidth_ = width;
    radarHeight_ = height;
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

void DynamicRadarAirwayGraph::GetDynamicFullPath(AirwayPointID sourceIdentity, AirwayPointID destinIdentity, std::vector<AirwayPoint> &path) {
    std::map<std::pair<AirwayPoint, AirwayPoint>, std::list<NodeInfo>> userWaypointMap;
    auto canSearch = [&](Edge edge, std::vector<Vertex> &previes) {
        if (blockSet_.find(edge) == blockSet_.end()) {
            return true;
        }
        RasterGraph rasterGraph(radarMask_, radarWidth_, radarHeight_);
        AirwayPoint &ap1 = airwayPointVector_[edge.first];
        AirwayPoint &ap2 = airwayPointVector_[edge.second];
        Pixel source = coordinateToPixel(ap1.x, ap1.y, worldFileInfo_);
        Pixel destin = coordinateToPixel(ap2.x, ap2.y, worldFileInfo_);
        std::vector<std::vector<Pixel>> nodes;
        rasterGraph.GetNodes(source, destin, nodes, 3);
        std::list<NodeInfo> infos;
        rasterGraph.GetPath(source, destin, nodes, infos);
        if (infos.empty()) {
            return false;
        } else {
            // 去掉首尾
            infos.pop_back();
            infos.pop_front();
            userWaypointMap[std::make_pair(ap1, ap2)] = infos;
            return true;
        }
    };
    std::vector<AirwayPoint> partialPath;
    GetPath(sourceIdentity, destinIdentity, partialPath, canSearch);
    path.clear();
    if (partialPath.size() > 0) {
        path.push_back(partialPath[0]);
    }
    for (int i = 1; i < partialPath.size(); i++) {
        auto &start = partialPath[i - 1];
        auto &end = partialPath[i];
        auto it = userWaypointMap.find(std::make_pair(start, end));
        if (it != userWaypointMap.end()) {
            for (auto &up: it->second) {
                const auto &ap = NodeInfoToAirwayPoint(up, worldFileInfo_);
                path.push_back(ap);
            }
        }
        path.push_back(end);
    }
};

//void DynamicRadarAirwayGraph::LogBlockAirpointSegment() {
//    for (auto &block: blockSet_) {
//        std::cout << "from:" << airwayPointVector_[block.small].airwayPointID << "  to:" << airwayPointVector_[block.big].airwayPointID << std::endl;
//    }
//}
