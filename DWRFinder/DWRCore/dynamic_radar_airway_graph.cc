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
#include <cassert>
#include "coordinate_convert.h"
#include "raster_graph.h"

namespace dwr {
    
const double kRadToDeg = 57.29577951308232;
//const double kDegToRad = .0174532925199432958;
    
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
    std::shared_ptr<Waypoint> userWaypoint(new Waypoint(kNoWaypointIdentifier, name, longitude, latitude));
    userWaypoint->coordinate = xy;
    userWaypoint->userWaypoint = true;
    return userWaypoint;
}

void DynamicRadarAirwayGraph::Build(const WorldFileInfo &world_file_info) {
    world_file_info_ = world_file_info;
    auto traverse_function = [&](const std::shared_ptr<Waypoint> &start_waypoint, const std::shared_ptr<Waypoint> &end_waypoint, GeoDistance d) {
        // 更新坐标
        if (start_waypoint->coordinate.x == kNoCoordinate) {
            LonLatToMerc(start_waypoint->location.longitude, start_waypoint->location.latitude, &start_waypoint->coordinate.x, &start_waypoint->coordinate.y);
        }
        if (end_waypoint->coordinate.x == kNoCoordinate) {
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
    if (start_waypoint->coordinate.x == kNoCoordinate) {
        LonLatToMerc(start_waypoint->location.longitude, start_waypoint->location.latitude, &start_waypoint->coordinate.x, &start_waypoint->coordinate.y);
    }
    Pixel start_pixel = CoordinateToPixel(start_waypoint->coordinate, world_file_info_);
    for (auto &neibor : start_waypoint->neibors) {
        auto end_waypoint = neibor.target.lock();
        // 如果是Build前end_waypoint是孤立的节点，则在Build中会遗漏该节点的坐标计算
        if (end_waypoint->coordinate.x == kNoCoordinate) {
            LonLatToMerc(end_waypoint->location.longitude, end_waypoint->location.latitude, &end_waypoint->coordinate.x, &end_waypoint->coordinate.y);
        }
        Pixel end_pixel = CoordinateToPixel(end_waypoint->coordinate, world_file_info_);
        Line linePixels = BresenhamLine(start_pixel, end_pixel);
        for (auto &point : linePixels) {
            pixel_to_edge_table_.emplace(point, UndirectedWaypointPair(start_waypoint, end_waypoint));
        }
    }
}

void DynamicRadarAirwayGraph::UpdateBlock(const std::shared_ptr<const char> &mask, int width, int height) {
    raster_graph_.SetRasterData(mask, width, height);
    // 更新阻塞集合
    block_set_.clear();
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            if (mask.get()[i * width + j] > 0) {
                Pixel pixel = Pixel(j, i);
                auto edge_iterator = pixel_to_edge_table_.find(pixel);
                if (edge_iterator != pixel_to_edge_table_.end()) {
                    block_set_.insert(edge_iterator->second);
                }
            }
        }
    }
}

WaypointPath DynamicRadarAirwayGraph::FindDynamicFullPath(WaypointIdentifier origin_identifier, WaypointIdentifier destination_identifier) const {
    auto can_search = [&](const WaypointPair &pair, WaypointPath &inserted_waypoints) {
        if (block_set_.find(UndirectedWaypointPair(pair)) == block_set_.end()) {
            if (pair.first->previous.lock() == nullptr) {
                return true;
            } else if (CosinTurnAngle(pair.first->previous.lock(), pair.first, pair.second) > 0) {
                return true;
            } else {
                return false;
            }
        }
        const std::shared_ptr<Waypoint> waypoint1 = pair.first;
        const std::shared_ptr<Waypoint> waypoint2 = pair.second;
        Pixel origin = CoordinateToPixel(waypoint1->coordinate, world_file_info_);
        Pixel destination = CoordinateToPixel(waypoint2->coordinate, world_file_info_);
        Pixel previous_origin = waypoint1->previous.lock() != nullptr ? CoordinateToPixel(waypoint1->previous.lock()->coordinate, world_file_info_) : kNoPixel;
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
    return FindPath(origin_identifier, destination_identifier, can_search);
};
    
std::vector<WaypointPath>
DynamicRadarAirwayGraph::FindKDynamicFullPath(WaypointIdentifier origin_identifier,
                                     WaypointIdentifier destination_identifier,
                                     int k) const {
    auto copied_graph = *this;
    std::function<WaypointPath (const DynamicRadarAirwayGraph &, WaypointIdentifier)> find_path = std::bind(&DynamicRadarAirwayGraph::FindDynamicFullPath, std::placeholders::_1, std::placeholders::_2, destination_identifier);
    return FindKPathInGraph(copied_graph, origin_identifier, destination_identifier, k, find_path);
}

//void DynamicRadarAirwayGraph::LogBlockAirpointSegment() {
//    for (auto &block: block_set_) {
//        std::cout << "from:" << waypointVector_[block.small].waypoint_identifier << "  to:" << waypointVector_[block.big].waypoint_identifier << std::endl;
//    }
//}
    
}
