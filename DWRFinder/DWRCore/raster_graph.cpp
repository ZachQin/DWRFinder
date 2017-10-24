//
//  raster_graph.cpp
//  DWRFinder
//
//  Created by ZachQin on 2017/3/16.
//  Copyright © 2017年 Zach. All rights reserved.
//

#include "raster_graph.hpp"
#include <queue>
#include <algorithm>
#include "graphics_utils.hpp"

namespace dwr {

std::vector<std::vector<Pixel>> RasterGraph::GetNodes(const Pixel &origin, const Pixel &destin, int segmentNumber, double verticalFactor) const {
    std::vector<std::vector<Pixel>> result;
    std::vector<Pixel> pixels = BresenhamLine(origin, destin);
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
    PixelDistance directDistance = Pixel::Distance(origin, destin);
    result = VerticalEquantLine(pixels[head], pixels[tail], segmentNumber, directDistance * verticalFactor);
    for (auto &node: result) {
        node.erase(std::remove_if(node.begin(), node.end(), [=](Pixel &p){
            return GetPixelValue(p) > 0;
        }), node.end());
    }
    return result;
}

bool RasterGraph::CheckLine(const Pixel &startPoint, const Pixel &endPoint) const {
    std::vector<Pixel> pixels = BresenhamLine(startPoint, endPoint);
    for (int i = 0; i < pixels.size(); i++) {
        if (GetPixelValue(pixels[i]) > 0) {
            return false;
        }
    }
    return true;
}
    
char RasterGraph::GetPixelValue(const Pixel &pixel) const {
    if (pixel.x >= 0 && pixel.x < width_ && pixel.y >= 0 && pixel.y < height_) {
        return rasterData_.get()[pixel.y * width_ + pixel.x];
    } else {
        return 0;
    }
}

std::vector<NodeInfo> RasterGraph::GetPath(const Pixel &origin, const Pixel &destin, const std::vector<std::vector<Pixel>> &nodeLevels, std::function<bool(const NodeInfoPair &pair)> canSearch) const {
    std::vector<NodeInfo> result;
    int levelSize = (int)nodeLevels.size();
    std::vector<std::vector<NodeInfo>> nodeInfoLevels(levelSize + 2);
    nodeInfoLevels[0] = {NodeInfo(0, Pixel::Distance(origin, destin), 0, origin, nullptr)};
    nodeInfoLevels[levelSize + 1] = {NodeInfo(max_distance, 0, levelSize + 1, destin, nullptr)};
    
    for (int i = 0; i < nodeLevels.size(); i++) {
        for (auto &px: nodeLevels[i]) {
            nodeInfoLevels[i + 1].push_back(NodeInfo(max_distance, Pixel::Distance(px, destin), i + 1, px, nullptr));
        }
    }
    auto nodeComp = [](const NodeInfo *n1, const NodeInfo *n2) {
        return *n1 > *n2;
    };
    std::priority_queue<NodeInfo *, std::vector<NodeInfo *>, decltype(nodeComp)> nodeQueue(nodeComp);
    nodeQueue.push(&nodeInfoLevels[0][0]);
    while (!nodeQueue.empty()) {
        NodeInfo *currentInfo = nodeQueue.top();
        PixelDistance dist = currentInfo->distance;
        Level level = currentInfo->level;
        Pixel u = currentInfo->pixel;
        nodeQueue.pop();
        
        if (u == destin) {
            break;
        }
        
        std::vector<NodeInfo> &nextLevels = nodeInfoLevels[level + 1];
        for (auto &v: nextLevels) {
            if (!canSearch(std::make_pair(*currentInfo, v))) {
                continue;
            }
            if (!CheckLine(u, v.pixel)) {
                continue;
            }
            PixelDistance distance_through_u = dist + Pixel::Distance(u, v.pixel);
            if (distance_through_u < v.distance) {
                v.distance = distance_through_u;
                v.previous = currentInfo;
                nodeQueue.push(&v);
            }
        }
    }
    
    NodeInfo *curNode = &nodeInfoLevels[levelSize + 1][0];
    // 如果找不到路径 直接返回空
    if (curNode->previous == nullptr) {
        return result;
    }
    while (curNode != nullptr) {
        result.push_back(*curNode);
        curNode = curNode->previous;
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

std::vector<NodeInfo> RasterGraph::GetPathWithAngle(const Pixel &origin, const Pixel &destin, const Pixel &previousOrigin) const {
    auto canSearch = [&](const NodeInfoPair &pair){
        // 搜索第一个节点需要判断参数中传入的节点位置
        if (pair.first.previous == nullptr) {
            if (previousOrigin.x == kNoPixel) {
                return true;
            } else {
                return CosinTurnAngle(previousOrigin, pair.first.pixel, pair.second.pixel) > 0;
            }
        } else {
            return CosinTurnAngle(pair.first.previous->pixel, pair.first.pixel, pair.second.pixel) > 0;
        }
    };
    std::vector<std::vector<Pixel>> nodes = GetNodes(origin, destin, 3);
    return GetPath(origin, destin, nodes, canSearch);
}

}
