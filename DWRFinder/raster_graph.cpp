//
//  raster_graph.cpp
//  DWRFinder
//
//  Created by ZachQin on 2017/3/16.
//  Copyright © 2017年 Zach. All rights reserved.
//

#include "raster_graph.hpp"
#include "graphics_utils.hpp"
#include <queue>
#include <algorithm>

namespace dwr {

std::vector<std::vector<Pixel>> RasterGraph::GetNodes(const Pixel &source, const Pixel &destin, int segmentNumber, double verticalFactor) const {
    std::vector<std::vector<Pixel>> result;
    std::vector<Pixel> pixels = BresenhamLine(source, destin);
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
    PixelDistance directDistance = Pixel::Distance(source, destin);
    result = VerticalEquantLine(pixels[head], pixels[tail], segmentNumber, directDistance * verticalFactor);
    for (auto &node: result) {
        node.erase(std::remove_if(node.begin(), node.end(), [=](Pixel &p){
            return GetPixelValue(p) > 0;
        }), node.end());
    }
    return result;
}

bool RasterGraph::CheckLine(const Pixel &startPoint, const Pixel &endPoint) {
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

std::vector<NodeInfo> RasterGraph::GetPath(Pixel source, Pixel destin, const std::vector<std::vector<Pixel>> &nodeLevels, std::function<bool(const NodeInfo &info)> canSearch) {
    std::vector<NodeInfo> result;
    int levelSize = (int)nodeLevels.size();
    std::vector<std::vector<NodeInfo>> nodeInfoLevels(levelSize + 2);
    nodeInfoLevels[0] = {NodeInfo(0, Pixel::Distance(source, destin),0, source, nullptr)};
    nodeInfoLevels[levelSize + 1] = {NodeInfo(max_distance, 0,levelSize, destin, nullptr)};
    
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
            if (!canSearch(v)) {
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

}
