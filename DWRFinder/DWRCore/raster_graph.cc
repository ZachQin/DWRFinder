//
//  raster_graph.cc
//  DWRFinder
//
//  Created by ZachQin on 2017/3/16.
//  Copyright © 2017年 Zach. All rights reserved.
//

#include "raster_graph.h"

#include <queue>
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

std::vector<PixelInfo> RasterGraph::FindPath(const Pixel &origin, const Pixel &destination, const std::vector<Line> &node_levels, const std::function<bool(const PixelInfoPair &pair)> &can_search) const {
    std::vector<PixelInfo> result;
    int level_size = static_cast<int>(node_levels.size());
    std::vector<std::vector<PixelInfo>> node_info_levels(level_size + 2);
    node_info_levels[0] = {PixelInfo(0, Pixel::Distance(origin, destination), 0, origin, nullptr)};
    node_info_levels[level_size + 1] = {PixelInfo(max_distance, 0, level_size + 1, destination, nullptr)};
    
    for (int i = 0; i < node_levels.size(); i++) {
        for (auto &px: node_levels[i]) {
            node_info_levels[i + 1].push_back(PixelInfo(max_distance, Pixel::Distance(px, destination), i + 1, px, nullptr));
        }
    }
    auto node_compare = [](const PixelInfo *n1, const PixelInfo *n2) {
        return *n1 > *n2;
    };
    std::priority_queue<PixelInfo *, std::vector<PixelInfo *>, decltype(node_compare)> node_queue(node_compare);
    node_queue.push(&node_info_levels[0][0]);
    while (!node_queue.empty()) {
        PixelInfo *current_info = node_queue.top();
        PixelDistance dist = current_info->distance;
        Level level = current_info->level;
        Pixel u = current_info->pixel;
        node_queue.pop();
        
        if (u == destination) {
            break;
        }
        
        std::vector<PixelInfo> &next_levels = node_info_levels[level + 1];
        for (auto &v: next_levels) {
            if (!can_search(std::make_pair(*current_info, v))) {
                continue;
            }
            if (!CheckLine(u, v.pixel)) {
                continue;
            }
            PixelDistance distance_through_u = dist + Pixel::Distance(u, v.pixel);
            if (distance_through_u < v.distance) {
                v.distance = distance_through_u;
                v.previous = current_info;
                node_queue.push(&v);
            }
        }
    }
    
    PixelInfo *current_node = &node_info_levels[level_size + 1][0];
    // 如果找不到路径 直接返回空
    if (current_node->previous == nullptr) {
        return result;
    }
    while (current_node != nullptr) {
        result.push_back(*current_node);
        current_node = current_node->previous;
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

std::vector<PixelInfo> RasterGraph::FindPathWithAngle(const Pixel &origin, const Pixel &destination, const Pixel &previous_origin) const {
    auto can_search = [&](const PixelInfoPair &pair){
        // 搜索第一个节点需要判断参数中传入的节点位置
        if (pair.first.previous == nullptr) {
            if (previous_origin.x == kNoPixel) {
                return true;
            } else {
                return CosinTurnAngle(previous_origin, pair.first.pixel, pair.second.pixel) > 0;
            }
        } else {
            return CosinTurnAngle(pair.first.previous->pixel, pair.first.pixel, pair.second.pixel) > 0;
        }
    };
    auto nodes = FetchCandidateLine(origin, destination, 3);
    return FindPath(origin, destination, nodes, can_search);
}

}
