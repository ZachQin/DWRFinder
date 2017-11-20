//
//  dynamic_radar_airway_graph.cc
//  DWRFinder
//
//  Created by ZachQin on 2017/3/8.
//  Copyright © 2017年 Zach. All rights reserved.
//

#include "dynamic_radar_airway_graph.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <iomanip>
#include "coordinate_convert.h"
#include "raster_graph.h"

namespace dwr {
    
const double kRadToDeg = 57.29577951308232;
    
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

std::string LonlatToString(double lon, double lat) {
    std::ostringstream text_stream;
    double longitude_degree = lon * kRadToDeg;
    text_stream << std::fixed << std::setprecision(2) << longitude_degree;
    if (longitude_degree >= 0) {
        text_stream << "E";
    } else {
        text_stream << "W";
    }
    double latitude_degree = lat * kRadToDeg;
    text_stream << latitude_degree;
    if (latitude_degree >= 0) {
        text_stream << "N";
    } else {
        text_stream << "S";
    }
    return text_stream.str();
}

std::shared_ptr<Waypoint> PixelToWaypoint(const Pixel &pixel, const WorldFileInfo &world_file_info) {
    GeoProj xy = PixelToCoordinate(pixel, world_file_info);
    double longitude, latitude;
    MercToLonLat(xy.x, xy.y, &longitude, &latitude);
    auto name = LonlatToString(longitude, latitude);
    auto userWaypoint = std::make_shared<Waypoint>(kNoWaypointIdentifier, name, longitude, latitude);
    userWaypoint->coordinate = xy;
    userWaypoint->user_waypoint = true;
    return userWaypoint;
}

void DynamicRadarAirwayGraph::Build(const WorldFileInfo &world_file_info) {
    world_file_info_ = world_file_info;
    auto traverse_function = [&](const std::shared_ptr<Waypoint> &start_waypoint,
                                 const std::shared_ptr<Waypoint> &end_waypoint, GeoDistance d) {
        // 更新坐标
        if (start_waypoint->coordinate == kNoCoordinate) {
            LonLatToMerc(start_waypoint->location.longitude, start_waypoint->location.latitude, &start_waypoint->coordinate.x, &start_waypoint->coordinate.y);
        }
        if (end_waypoint->coordinate == kNoCoordinate) {
            LonLatToMerc(end_waypoint->location.longitude, end_waypoint->location.latitude, &end_waypoint->coordinate.x, &end_waypoint->coordinate.y);
        }
        Pixel start_pixel = CoordinateToPixel(start_waypoint->coordinate, world_file_info);
        Pixel end_pixel = CoordinateToPixel(end_waypoint->coordinate, world_file_info);
        Line linePixels = BresenhamLine(start_pixel, end_pixel);
        for (auto &point : linePixels) {
            pixel_to_edge_table_.emplace(point, UndirectedWaypointPair(start_waypoint, end_waypoint));
        }
    };
    this->ForEach(traverse_function);
}

void DynamicRadarAirwayGraph::SingleBuild(WaypointIdentifier identifier) {
    auto start_waypoint = WaypointFromIdentifier(identifier);
    if (start_waypoint == nullptr) {
        return;
    }
    if (start_waypoint->coordinate == kNoCoordinate) {
        LonLatToMerc(start_waypoint->location.longitude, start_waypoint->location.latitude, &start_waypoint->coordinate.x, &start_waypoint->coordinate.y);
    }
    Pixel start_pixel = CoordinateToPixel(start_waypoint->coordinate, world_file_info_);
    for (auto &neibor : start_waypoint->neibors) {
        auto end_waypoint = neibor.target.lock();
        // 如果是Build前end_waypoint是孤立的节点，则在Build中会遗漏该节点的坐标计算
        if (end_waypoint->coordinate == kNoCoordinate) {
            LonLatToMerc(end_waypoint->location.longitude, end_waypoint->location.latitude, &end_waypoint->coordinate.x, &end_waypoint->coordinate.y);
        }
        Pixel end_pixel = CoordinateToPixel(end_waypoint->coordinate, world_file_info_);
        Line linePixels = BresenhamLine(start_pixel, end_pixel);
        for (auto &point : linePixels) {
            pixel_to_edge_table_.emplace(point, UndirectedWaypointPair(start_waypoint, end_waypoint));
        }
    }
}

void DynamicRadarAirwayGraph::UpdateBlock(char *mask, int width, int height) {
    raster_graph_.SetRasterData(mask, width, height);
    // 更新阻塞集合
    block_set_.clear();
    raster_graph_.ForEach([&](int x, int y, char value){
        if (value > 0) {
            Pixel pixel = Pixel(x, y);
            auto edge_iterator = pixel_to_edge_table_.find(pixel);
            if (edge_iterator != pixel_to_edge_table_.end()) {
                block_set_.insert(edge_iterator->second);
            }
        }
    });
}

WaypointPath
DynamicRadarAirwayGraph::FindDynamicFullPath(WaypointIdentifier origin_identifier,
                                             WaypointIdentifier destination_identifier,
                                             const std::function<bool(const WaypointPair &waypoint_pair,
                                                                      const WaypointInfoPair &info_pair,
                                                                      std::vector<std::shared_ptr<Waypoint>> &inserted_waypoints)> &can_search) const {
    auto inner_can_search = [&](const WaypointPair &waypoint_pair,
                                const WaypointInfoPair &info_pair,
                                std::vector<std::shared_ptr<Waypoint>> &inserted_waypoints) {
        if (!can_search(waypoint_pair, info_pair, inserted_waypoints)) {
            return false;
        }
        const std::shared_ptr<Waypoint> &waypoint1 = waypoint_pair.first;
        const std::shared_ptr<Waypoint> &waypoint2 = waypoint_pair.second;
        const WaypointInfo &waypoint_info1 = info_pair.first;
        
        if (block_set_.find(UndirectedWaypointPair(waypoint_pair)) == block_set_.end()) {
            if (waypoint_info1.previous.lock() == nullptr) {
                return true;
            } else if (Waypoint::CosinTurnAngle(*waypoint_info1.previous.lock(), *waypoint1, *waypoint2) > 0) {
                return true;
            } else {
                return false;
            }
        }
        
        const Pixel origin = CoordinateToPixel(waypoint1->coordinate, world_file_info_);
        const Pixel destination = CoordinateToPixel(waypoint2->coordinate, world_file_info_);
        const Pixel previous_origin = waypoint_info1.previous.lock() != nullptr ? CoordinateToPixel(waypoint_info1.previous.lock()->coordinate, world_file_info_) : kNoPixel;
        PixelPath pixel_path = raster_graph_.FindPathWithAngle(origin, destination, previous_origin);
        if (pixel_path.empty()) {
            return false;
        } else {
            // 去掉首尾
            inserted_waypoints.resize(pixel_path.size() - 2);
            std::transform(pixel_path.begin() + 1, pixel_path.end() - 1, inserted_waypoints.begin(), [&](const Pixel &pixel){
                return PixelToWaypoint(pixel, world_file_info_);
            });
            return true;
        }
    };
    return FindPath(origin_identifier, destination_identifier, inner_can_search);
};
    
std::vector<WaypointPath>
DynamicRadarAirwayGraph::FindKDynamicFullPath(WaypointIdentifier origin_identifier,
                     WaypointIdentifier destination_identifier,
                     int k) const {
    auto find_path = [&](const std::shared_ptr<Waypoint> &spur_waypoint, const std::shared_ptr<Waypoint> &destination_waypoint, const std::set<WaypointPair> &block_set) {
        auto can_search = [block_set](const dwr::WaypointPair &p,
                                      const dwr::WaypointInfoPair &,
                                      std::vector<std::shared_ptr<dwr::Waypoint>> &inserted_waypoints) {return block_set.find(p) == block_set.end();};
        return FindDynamicFullPath(spur_waypoint->waypoint_identifier, destination_waypoint->waypoint_identifier, can_search);
    };
    return FindKPath(origin_identifier, destination_identifier, k, find_path);
}
    
}
