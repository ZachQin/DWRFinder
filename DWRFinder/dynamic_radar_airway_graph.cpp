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
#include <cassert>
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
    
    Pixel CoordinateToPixel(const GeoProj &xy, const WorldFileInfo &w) {
        Pixel pixel;
        pixel.x = (w.E * xy.x - w.B * xy.y + w.B * w.F - w.E * w.C) / (w.A * w.E - w.D * w.B);
        pixel.y = (-w.D * xy.x + w.A * xy.y + w.D * w.C - w.A * w.F) / (w.A * w.E - w.D * w.B);
        return pixel;
    }
    
    GeoProj PixelToCoordinate(const Pixel &pixel, const WorldFileInfo &w) {
        GeoProj xy;
        xy.x = w.A * pixel.x + w.B * pixel.y + w.C;
        xy.y = w.D * pixel.x + w.E * pixel.y + w.F;
        return xy;
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
    
    std::string LonlatToString(double lon, double lat) {
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
    
    std::shared_ptr<Waypoint> NodeInfoToWaypoint(const NodeInfo &info, const WorldFileInfo &w) {
        GeoProj xy = PixelToCoordinate(info.pixel, w);
        double lon, lat;
        MercToLonLat(xy.x, xy.y, &lon, &lat);
        auto name = LonlatToString(lon, lat);
        std::shared_ptr<Waypoint> userWaypoint(new Waypoint(kNoAirwaypointID, name, lon, lat));
        userWaypoint->coordinate = xy;
        userWaypoint->userWaypoint = true;
        return userWaypoint;
    }
    
    void DynamicRadarAirwayGraph::Prebuild(const WorldFileInfo &worldFileInfo) {
        worldFileInfo_ = worldFileInfo;
        auto traverseFunction = [&](const std::shared_ptr<Waypoint> &startWpt, const std::shared_ptr<Waypoint> &endWpt, GeoDistance d) {
            // 更新坐标
            if (startWpt->coordinate.x == kNoCoordinate) {
                LonLatToMerc(startWpt->location.longitude, startWpt->location.latitude, &startWpt->coordinate.x, &startWpt->coordinate.y);
            }
            if (endWpt->coordinate.x == kNoCoordinate) {
                LonLatToMerc(endWpt->location.longitude, endWpt->location.latitude, &endWpt->coordinate.x, &endWpt->coordinate.y);
            }
            Pixel startPixel = CoordinateToPixel(startWpt->coordinate, worldFileInfo);
            Pixel endPixel = CoordinateToPixel(endWpt->coordinate, worldFileInfo);
            std::vector<Pixel> linePixels = BresenhamLine(startPixel, endPixel);
            for (auto &point : linePixels) {
                auto upair = UndirectedWaypointPair(startWpt, endWpt);
                pixelToEdgeTable_.insert(std::make_pair(point, upair));
            }
        };
        this->ForEach(traverseFunction);
    }
    
    void DynamicRadarAirwayGraph::Build(WaypointID identifier) {
        auto startWpt = WaypointFromID(identifier);
        if (startWpt == nullptr) {
            return;
        }
        if (startWpt->coordinate.x == kNoCoordinate) {
            LonLatToMerc(startWpt->location.longitude, startWpt->location.latitude, &startWpt->coordinate.x, &startWpt->coordinate.y);
        }
        Pixel startPixel = CoordinateToPixel(startWpt->coordinate, worldFileInfo_);
        for (auto &neibor : startWpt->neibors) {
            auto endWpt = neibor.target.lock();
            // 如果是Prebuild前endWpt是孤立的节点，则在Prebuild中会遗漏该节点的坐标计算
            if (endWpt->coordinate.x == kNoCoordinate) {
                LonLatToMerc(endWpt->location.longitude, endWpt->location.latitude, &endWpt->coordinate.x, &endWpt->coordinate.y);
            }
            Pixel endPixel = CoordinateToPixel(endWpt->coordinate, worldFileInfo_);
            std::vector<Pixel> linePixels = BresenhamLine(startPixel, endPixel);
            for (auto &point : linePixels) {
                auto upair = UndirectedWaypointPair(startWpt, endWpt);
                pixelToEdgeTable_.insert(std::make_pair(point, upair));
            }
        }
    }
    
    void DynamicRadarAirwayGraph::UpdateBlock(const std::shared_ptr<const char> &mask, int width, int height) {
        rasterGraph_.SetRasterData(mask, width, height);
        // 更新阻塞集合
        blockSet_.clear();
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                if (mask.get()[i * width + j] > 0) {
                    Pixel pixel = Pixel(j, i);
                    auto edgeIter = pixelToEdgeTable_.find(pixel);
                    if (edgeIter != pixelToEdgeTable_.end()) {
                        blockSet_.insert(edgeIter->second);
                    }
                }
            }
        }
    }
    
    std::vector<std::shared_ptr<Waypoint>> DynamicRadarAirwayGraph::GetDynamicFullPath(WaypointID originIdentifier, WaypointID destinIdentifier) {
        auto canSearch = [&](const WaypointPair &pair, std::vector<std::shared_ptr<Waypoint>> &insertedWaypoints) {
            if (blockSet_.find(UndirectedWaypointPair(pair)) == blockSet_.end()) {
                if (pair.first->previous.lock() == nullptr) {
                    return true;
                } else if (CosinTurnAngle(pair.first->previous.lock(), pair.first, pair.second) > 0) {
                    return true;
                } else {
                    return false;
                }
            }
            const std::shared_ptr<Waypoint> ap1 = pair.first;
            const std::shared_ptr<Waypoint> ap2 = pair.second;
            Pixel origin = CoordinateToPixel(ap1->coordinate, worldFileInfo_);
            Pixel destin = CoordinateToPixel(ap2->coordinate, worldFileInfo_);
            Pixel previousOrigin = ap1->previous.lock() != nullptr ? CoordinateToPixel(ap1->previous.lock()->coordinate, worldFileInfo_) : Pixel(kNoPixel, kNoPixel);
            std::vector<NodeInfo> infos = rasterGraph_.GetPathWithAngle(origin, destin, previousOrigin);
            if (infos.empty()) {
                return false;
            } else {
                // 去掉首尾
                insertedWaypoints.resize(infos.size() - 2);
                std::transform(infos.begin() + 1, infos.end() - 1, insertedWaypoints.begin(), [&](NodeInfo info){
                    return NodeInfoToWaypoint(info, worldFileInfo_);
                });
                return true;
            }
        };
        std::vector<std::shared_ptr<Waypoint>> fullPath = GetPath(originIdentifier, destinIdentifier, canSearch);
        return fullPath;
    };
    
    //void DynamicRadarAirwayGraph::LogBlockAirpointSegment() {
    //    for (auto &block: blockSet_) {
    //        std::cout << "from:" << waypointVector_[block.small].waypointID << "  to:" << waypointVector_[block.big].waypointID << std::endl;
    //    }
    //}
    
}
