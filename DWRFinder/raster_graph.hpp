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
#include <math.h>
#include "raster_type.hpp"

namespace dwr {

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
