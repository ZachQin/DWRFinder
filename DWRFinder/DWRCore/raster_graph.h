//
//  raster_graph.h
//  DWRFinder
//
//  Created by ZachQin on 2017/3/16.
//  Copyright © 2017年 Zach. All rights reserved.
//

#ifndef raster_graph_h
#define raster_graph_h

#include <functional>
#include <math.h>
#include "raster_type.h"

namespace dwr {
    
class RasterGraph {
public:
    RasterGraph() : raster_data_(nullptr), width_(0), height_(0) {};
    
    RasterGraph(char *raster_data, int width, int height): raster_data_(raster_data), width_(width), height_(height) {};
        
    std::vector<Line> FetchCandidateLine(const Pixel &origin,
                                         const Pixel &destination,
                                         int segment_number,
                                         double vertical_factor = 0.5
                                         ) const;
    
    PixelPath FindPathWithAngle(const Pixel &origin,
                                const Pixel &destination,
                                const Pixel &previous_origin = kNoPixel
                                ) const;
    
    
    char GetPixelValue(const Pixel &pixel) const;
    
    void ForEach(const std::function<void(int x, int y, char value)> &traverse_function) {
        for (int i = 0; i < height_; i++)
            for (int j = 0; j < width_; j++)
                traverse_function(j, i, raster_data_.get()[i * width_ + j]);
    }
    
    void SetRasterData(char *raster_data, int width, int height) {
        raster_data_.reset(raster_data);
        width_ = width;
        height_ = height;
    }
    
private:
    std::unique_ptr<char> raster_data_;
    int width_;
    int height_;
    bool CheckLine(const Pixel &start_pixel, const Pixel &end_pixel) const;
};
    
double CosinTurnAngle(const Pixel &previous, const Pixel &current, const Pixel &next);

PixelPath FindPath(const Pixel &origin,
                   const Pixel &destination,
                   const std::vector<Line> &node_levels,
                   const std::function<bool(const PixelPair &pixel_pair, const PixelInfoPair &info_pair)> &can_search =
                   [](const PixelPair &pixel_pair, const PixelInfoPair &info_pair){return true;}
                   );
}
#endif /* raster_graph_h */
