//
//  raster_graph.cc
//  DWRFinder
//
//  Created by ZachQin on 2017/3/16.
//  Copyright © 2017年 Zach. All rights reserved.
//

#include "raster_graph.h"

#include <queue>
#include <map>
#include <algorithm>
#include "graphics_utils.h"

namespace dwr {

std::vector<Line> RasterGraph::FetchCandidateLine(const Pixel &origin, const Pixel &destination, int segment_number, double vertical_factor) const {
    std::vector<Line> result;
    Line pixels = BresenhamLine(origin, destination);
    int head = 0, tail = (int)pixels.size() - 1;
    while (head < pixels.size() && GetPixelValue(pixels[head]) == 0) {
        head++;
    }
    while (tail >= 0 && GetPixelValue(pixels[tail]) == 0) {
        tail--;
    }
    if (head >= tail) {
        return result;
    }
    PixelDistance directDistance = Pixel::Distance(origin, destination);
    result = VerticalEquantLine(pixels[head], pixels[tail], segment_number, directDistance * vertical_factor);
    for (auto &node: result) {
        node.erase(std::remove_if(node.begin(), node.end(), [=](Pixel &p){
            return GetPixelValue(p) > 0;
        }), node.end());
    }
    return result;
}

bool RasterGraph::CheckLine(const Pixel &start_pixel, const Pixel &end_pixel) const {
    Line pixels = BresenhamLine(start_pixel, end_pixel);
    for (int i = 0; i < pixels.size(); i++) {
        if (GetPixelValue(pixels[i]) > 0) {
            return false;
        }
    }
    return true;
}
    
char RasterGraph::GetPixelValue(const Pixel &pixel) const {
    if (pixel.x >= 0 && pixel.x < width_ && pixel.y >= 0 && pixel.y < height_) {
        return raster_data_.get()[pixel.y * width_ + pixel.x];
    } else {
        return 0;
    }
}

PixelPath RasterGraph::FindPath(const Pixel &origin, const Pixel &destination, const std::vector<Line> &node_levels, const std::function<bool(const PixelInfoPair &pair)> &can_search) const {
    PixelPath result;
    int level_size = static_cast<int>(node_levels.size());
    std::map<Pixel, PixelInfo> info_map;
    info_map[origin] = PixelInfo(0, Pixel::Distance(origin, destination), 0, origin, kNoPixel);
    info_map[destination] = PixelInfo(max_distance, 0, level_size + 1, destination, kNoPixel);
    for (int i = 0; i < node_levels.size(); i++) {
        for (auto &px: node_levels[i]) {
            info_map[px] = PixelInfo(max_distance, Pixel::Distance(px, destination), i + 1, px, kNoPixel);
        }
    }
    auto node_compare = [](const PixelInfo *n1, const PixelInfo *n2) {
        return *n1 > *n2;
    };
    std::priority_queue<PixelInfo *, std::vector<PixelInfo *>, decltype(node_compare)> node_queue(node_compare);
    node_queue.push(&info_map[origin]);
    while (!node_queue.empty()) {
        PixelInfo *current_info = node_queue.top();
        PixelDistance dist = current_info->distance;
        Level level = current_info->level;
        Pixel u = current_info->pixel;
        node_queue.pop();
        
        if (u == destination) {
            break;
        }
        
        const Line &next_level = level != level_size ? node_levels[level] : Line{destination};
        for (auto &v: next_level) {
            auto &v_info = info_map[v];
            if (!can_search(std::make_pair(*current_info, v_info))) {
                continue;
            }
            if (!CheckLine(u, v)) {
                continue;
            }
            PixelDistance distance_through_u = dist + Pixel::Distance(u, v);
            if (distance_through_u < v_info.distance) {
                v_info.distance = distance_through_u;
                v_info.previous = current_info->pixel;
                node_queue.push(&v_info);
            }
        }
    }
    
    PixelInfo *current_node = &info_map[destination];
    // 如果找不到路径 直接返回空
    if (current_node->previous == kNoPixel) {
        return result;
    }
    while (true) {
        result.push_back(current_node->pixel);
        if (current_node->previous == kNoPixel) {
            break;
        }
        current_node = &info_map[current_node->previous];
    }
    std::reverse(result.begin(), result.end());
    return result;
}

double CosinTurnAngle(const Pixel &previous, const Pixel &current, const Pixel &next) {
    double pc_x = current.x - previous.x;
    double pc_y = current.y - previous.y;
    double cn_x = next.x - current.x;
    double cn_y = next.y - current.y;
    return (pc_x * cn_x + pc_y * cn_y) / (sqrt(pc_x * pc_x + pc_y * pc_y) * sqrt(cn_x * cn_x + cn_y * cn_y));
}

PixelPath RasterGraph::FindPathWithAngle(const Pixel &origin, const Pixel &destination, const Pixel &previous_origin) const {
    auto can_search = [&](const PixelInfoPair &pair){
        // 搜索第一个节点需要判断参数中传入的节点位置
        if (pair.first.previous == kNoPixel) {
            if (previous_origin == kNoPixel) {
                return true;
            } else {
                return CosinTurnAngle(previous_origin, pair.first.pixel, pair.second.pixel) > 0;
            }
        } else {
            return CosinTurnAngle(pair.first.previous, pair.first.pixel, pair.second.pixel) > 0;
        }
    };
    auto nodes = FetchCandidateLine(origin, destination, 3);
    if (nodes.empty()) {
        return Line();
    }
    return FindPath(origin, destination, nodes, can_search);
}

}
