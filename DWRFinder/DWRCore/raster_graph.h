//
//  raster_graph.h
//  DWRFinder
//
//  Created by ZachQin on 2017/3/16.
//  Copyright © 2017年 Zach. All rights reserved.
//

#ifndef raster_graph_h
#define raster_graph_h

#include <stdio.h>
#include <functional>
#include <math.h>
#include "raster_type.h"

namespace dwr {

class RasterGraph {
public:
    RasterGraph() : raster_data_(nullptr), width_(0), height_(0) {};
    RasterGraph(const std::shared_ptr<const char> &raster_data, int width, int height): raster_data_(raster_data), width_(width), height_(height) {};
    void ResetRaster(const std::shared_ptr<const char> &raster_data, int width, int height) {raster_data_ = raster_data; width_ = width; height_ = height;};
    std::vector<std::vector<Pixel>> GetNodes(const Pixel &origin, const Pixel &destination, int segment_number, double vertical_factor = 0.5) const;
    std::vector<NodeInfo> GetPath(const Pixel &origin, const Pixel &destination, const std::vector<std::vector<Pixel>> &node_levels, std::function<bool(const NodeInfoPair &info)> can_search = [](const NodeInfoPair &info){return true;}) const;
    std::vector<NodeInfo> GetPathWithAngle(const Pixel &origin, const Pixel &destination, const Pixel &previous_origin = {kNoPixel, kNoPixel}) const;
    char GetPixelValue(const Pixel &pixel) const;
    void SetRasterData(std::shared_ptr<const char> raster_data, int width, int height) {
        raster_data_ = raster_data;
        width_ = width;
        height_ = height;
    }

private:
    std::shared_ptr<const char> raster_data_;
    int width_;
    int height_;
    bool CheckLine(const Pixel &start_pixel, const Pixel &end_pixel) const;
};

}
#endif /* raster_graph_h */
