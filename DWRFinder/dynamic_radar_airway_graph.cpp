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

void DynamicRadarAirwayGraph::prebuild(const WorldFileInfo &worldFileInfo) {
    for (Vertex startVertex = 0; startVertex < adjacencyList_.size(); startVertex++) {
        auto neighbors = adjacencyList_[startVertex];
        for (int index = 0; index < neighbors.size(); index++) {
            auto neighbor = neighbors[index];
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

void DynamicRadarAirwayGraph::UpdateBlock(const char *mask, size_t width, size_t height) {
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

//void DynamicRadarAirwayGraph::LogBlockAirpointSegment() {
//    for (auto &block: blockSet_) {
//        std::cout << "from:" << airwayPointVector_[block.small].airwayPointID << "  to:" << airwayPointVector_[block.big].airwayPointID << std::endl;
//    }
//}
