//
//  raster_graph.hpp
//  DWRFinder
//
//  Created by ZachQin on 2017/3/16.
//  Copyright © 2017年 Zach. All rights reserved.
//

#ifndef raster_graph_hpp
#define raster_graph_hpp

#include <stdio.h>
#include <list>
#include <functional>
#include "graphics_utils.hpp"

namespace dwr {

typedef int Level;

const Distance max_distance = std::numeric_limits<Distance>::max();

struct NodeInfo {
    Distance distance;
    Distance heuristic;
    Level level;
    Pixel pixel;
    NodeInfo *previous;
    
    NodeInfo(Distance dist, Distance heur, Level l, Pixel px, NodeInfo *pre): distance(dist), heuristic(heur), level(l), pixel(px), previous(pre){};
    
    bool operator > (const NodeInfo &p) const {
        if (distance == max_distance) {return true;}
        if (p.distance == max_distance) {return false;}
        return distance + heuristic > p.distance + p.heuristic;
    }
};

class RasterGraph {
public:
    RasterGraph(const char *rasterData, int width, int height): rasterData_(rasterData), width_(width), height_(height) {};
    void ResetRaster(const char *rasterData, int width, int height) {rasterData_ = rasterData; width_ = width; height_ = height;};
    void GetNodes(Pixel source, Pixel destin, std::vector<std::vector<Pixel>> &nodes, int segmentNumber, double verticalFactor = 0.5);
    void GetPath(Pixel source, Pixel destin, std::vector<std::vector<Pixel>> &nodeLevels, std::list<NodeInfo> &nodeInfos, std::function<bool(const NodeInfo &info)> canSearch = [](const NodeInfo &info){return true;});

private:
    const char *rasterData_;
    int width_;
    int height_;
    bool CheckLine(Pixel startPoint, Pixel endPoint);
};

}
#endif /* raster_graph_hpp */
