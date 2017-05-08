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
#include <functional>
#include <math.h>
#include "raster_type.hpp"

namespace dwr {

class RasterGraph {
public:
    RasterGraph() : rasterData_(nullptr), width_(0), height_(0) {};
    RasterGraph(const char *rasterData, int width, int height): rasterData_(rasterData), width_(width), height_(height) {};
    void ResetRaster(const std::shared_ptr<const char> &rasterData, int width, int height) {rasterData_ = rasterData; width_ = width; height_ = height;};
    std::vector<std::vector<Pixel>> GetNodes(const Pixel &origin, const Pixel &destin, int segmentNumber, double verticalFactor = 0.5) const;
    std::vector<NodeInfo> GetPath(Pixel origin, Pixel destin, const std::vector<std::vector<Pixel>> &nodeLevels, std::function<bool(const NodeInfo &info)> canSearch = [](const NodeInfo &info){return true;}) const;
    char GetPixelValue(const Pixel &pixel) const;
    void SetRasterData(std::shared_ptr<const char> rasterData, int width, int height) {
        rasterData_ = rasterData;
        width_ = width;
        height_ = height;
    }

private:
    std::shared_ptr<const char> rasterData_;
    int width_;
    int height_;
    bool CheckLine(const Pixel &startPoint, const Pixel &endPoint) const;
};

}
#endif /* raster_graph_hpp */
