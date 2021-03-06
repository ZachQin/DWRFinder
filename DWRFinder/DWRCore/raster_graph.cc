//
//  raster_graph.cc
//  DWRFinder
//
//  Created by ZachQin on 2017/3/16.
//  Copyright © 2017年 Zach. All rights reserved.
//

#include "raster_graph.h"

#include <queue>
#include <unordered_map>
#include <algorithm>
#include <utility>
#include <vector>

#include "Utils/graphics_utils.h"

namespace dwr {

std::vector<Line>
RasterGraph::FetchCandidateLine(const Pixel &origin,
                                const Pixel &destination,
                                int segment_number,
                                double vertical_factor) const {
    std::vector<Line> result;
    Line pixels = BresenhamLine(origin, destination);
    int head = 0, tail = static_cast<int>(pixels.size()) - 1;
    while (head < pixels.size() && GetPixelValue(pixels[head]) == 0) {
        head++;
    }
    while (tail >= 0 && GetPixelValue(pixels[tail]) == 0) {
        tail--;
    }
    if (head >= tail) {
        return result;
    }
    PixelDistance direct_distance = Pixel::Distance(origin, destination);
    result = VerticalEquantLine(pixels[head],
                                pixels[tail],
                                segment_number,
                                direct_distance * vertical_factor);
    for (auto &node : result) {
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

void RasterGraph::ForEach(const std::function<void(int x, int y, char value)> &traverse_function) const {
    for (int i = 0; i < height_; i++)
        for (int j = 0; j < width_; j++)
            traverse_function(j, i, raster_data_.get()[i * width_ + j]);
}

char RasterGraph::GetPixelValue(const Pixel &pixel) const {
    if (pixel.x >= 0 && pixel.x < width_ && pixel.y >= 0 && pixel.y < height_) {
        return raster_data_.get()[pixel.y * width_ + pixel.x];
    } else {
        return 0;
    }
}

PixelPath
RasterGraph::FindPathWithAngle(const Pixel &origin,
                               const Pixel &destination,
                               const Pixel &previous_origin) const {
    auto can_search = [&](const PixelPair &pixel_pair, const PixelInfoPair &info_pair){
        bool result = true;
        // 搜索第一个节点需要判断参数中传入的节点位置
        if (info_pair.first.previous == kNoPixel) {
            if (previous_origin == kNoPixel) {
                result = true;
            } else {
                result = Pixel::CosinTurnAngle(previous_origin, pixel_pair.first, pixel_pair.second) > 0;
            }
        } else {
            result = Pixel::CosinTurnAngle(info_pair.first.previous, pixel_pair.first, pixel_pair.second) > 0;
        }
        return result && CheckLine(pixel_pair.first, pixel_pair.second);
    };
    auto nodes = FetchCandidateLine(origin, destination, 3);
    if (nodes.empty()) {
        return Line();
    }
    return FindPath(origin, destination, nodes, can_search);
}

static PixelDistance HeuristicDistance(const Pixel &p1, const Pixel &p2) {
    return Pixel::Distance(p1, p2) * 0.9;
}

PixelPath
RasterGraph::FindPath(const Pixel &origin,
                      const Pixel &destination,
                      const std::vector<Line> &node_levels,
                      const std::function<bool(const PixelPair &, const PixelInfoPair &)> &can_search) {
    PixelPath result;
    int level_size = static_cast<int>(node_levels.size());
    std::unordered_map<Pixel, PixelInfo> info_map;
    info_map[origin] = PixelInfo(0, HeuristicDistance(origin, destination), 0, kNoPixel);
    info_map[destination] = PixelInfo(kMaxPixelDistance, 0, level_size + 1, kNoPixel);
    for (int i = 0; i < node_levels.size(); i++) {
        for (auto &px : node_levels[i]) {
            info_map[px] = PixelInfo(kMaxPixelDistance, kMaxPixelDistance, i + 1, kNoPixel);
        }
    }
    auto node_compare = [&info_map](const Pixel pixel1, const Pixel pixel2) {
        return info_map[pixel1].estimated_distance > info_map[pixel2].estimated_distance;
    };
    std::priority_queue<Pixel, std::vector<Pixel>, decltype(node_compare)> node_queue(node_compare);
    node_queue.push(origin);
    while (!node_queue.empty()) {
        Pixel u = node_queue.top();
        PixelInfo &current_info = info_map[u];
        PixelDistance dist = current_info.actual_distance;
        Level level = current_info.level;
        node_queue.pop();
        if (u == destination) {
            break;
        }
        const Line &next_level = level != level_size ? node_levels[level] : Line{destination};
        for (auto &v : next_level) {
            auto &v_info = info_map[v];
            if (!can_search(std::make_pair(u, v), std::make_pair(current_info, v_info))) {
                continue;
            }
            PixelDistance distance_through_u = dist + Pixel::Distance(u, v);
            if (distance_through_u < v_info.actual_distance) {
                v_info.actual_distance = distance_through_u;
                v_info.previous = u;
                v_info.estimated_distance = v_info.actual_distance + HeuristicDistance(v, destination);
                node_queue.push(v);
            }
        }
    }
    Pixel current_pixel = destination;
    // 如果找不到路径 直接返回空
    if (info_map[current_pixel].previous == kNoPixel) {
        return result;
    }
    while (true) {
        result.push_back(current_pixel);
        PixelInfo &current_info = info_map[current_pixel];
        if (current_info.previous == kNoPixel) {
            break;
        }
        current_pixel = current_info.previous;
    }
    std::reverse(result.begin(), result.end());
    return result;
}

}  // namespace dwr
