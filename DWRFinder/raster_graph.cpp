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
#include <map>

void RasterGraph::GetNodes(Pixel source, Pixel destin, std::vector<std::vector<Pixel>> &nodes, int segmentNumber, double verticalFactor) {
    std::vector<Pixel> pixels;
    BresenhamLine(source, destin, pixels);
    int head = 0, tail = (int)pixels.size() - 1;
    while (head < pixels.size() && pixels[head].getPixelValue(rasterData_, width_, height_) == 0) {
        head++;
    }
    while (tail >= 0 && pixels[tail].getPixelValue(rasterData_, width_, height_) == 0) {
        tail--;
    }
    if (head >= tail) {
        return;
    }
    Distance directDistance = source.DistanceTo(destin);
    
    VerticalEquantLine(pixels[head], pixels[tail], segmentNumber, directDistance * verticalFactor, nodes);
    for (auto &node: nodes) {
        node.erase(std::remove_if(node.begin(), node.end(), [=](Pixel &p){
            return p.getPixelValue(rasterData_, width_, height_) > 0;
        }), node.end());
    }
}

bool RasterGraph::CheckLine(Pixel startPoint, Pixel endPoint) {
    std::vector<Pixel> pixels;
    BresenhamLine(startPoint, endPoint, pixels);
    for (int i = 0; i < pixels.size(); i++) {
        if (pixels[i].getPixelValue(rasterData_, width_, height_) > 0) {
            return false;
        }
    }
    return true;
}

void RasterGraph::GetPath(Pixel source, Pixel destin, std::vector<std::vector<Pixel>> &nodeLevels, std::list<NodeInfo> &nodeInfos, std::function<bool(const NodeInfo &info)> canSearch) {
    int levelSize = (int)nodeLevels.size();
    std::vector<std::vector<NodeInfo>> nodeInfoLevels(levelSize + 2);
    nodeInfoLevels[0] = {NodeInfo(0, source.DistanceTo(destin),0, source, nullptr)};
    nodeInfoLevels[levelSize + 1] = {NodeInfo(max_distance, 0,levelSize, destin, nullptr)};
    
    for (int i = 0; i < nodeLevels.size(); i++) {
        for (auto &px: nodeLevels[i]) {
            nodeInfoLevels[i + 1].push_back(NodeInfo(max_distance, px.DistanceTo(destin), i + 1, px, nullptr));
        }
    }
    auto nodeComp = [](const NodeInfo *n1, const NodeInfo *n2) {
        return *n1 > *n2;
    };
    std::priority_queue<NodeInfo *, std::vector<NodeInfo *>, decltype(nodeComp)> nodeQueue(nodeComp);
    nodeQueue.push(&nodeInfoLevels[0][0]);
    while (!nodeQueue.empty()) {
        NodeInfo *currentInfo = nodeQueue.top();
        Distance dist = currentInfo->distance;
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
            Distance distance_through_u = dist + u.DistanceTo(v.pixel);
            if (distance_through_u < v.distance) {
                v.distance = distance_through_u;
                v.previous = currentInfo;
                nodeQueue.push(&v);
            }
        }
    }
    
    nodeInfos.clear();
    NodeInfo *curNode = &nodeInfoLevels[levelSize + 1][0];
    // 如果找不到路径 直接返回空
    if (curNode->previous == nullptr) {
        return;
    }
    while (curNode != nullptr) {
        nodeInfos.push_front(*curNode);
        curNode = curNode->previous;
    }
}
