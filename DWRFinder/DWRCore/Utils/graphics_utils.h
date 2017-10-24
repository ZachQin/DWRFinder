//
//  GraphicsUtils.h
//  DWRFinder
//
//  Created by ZachQin on 2017/3/16.
//  Copyright © 2017年 Zach. All rights reserved.
//

#ifndef graphics_utils_h
#define graphics_utils_h

#include <vector>
#include "raster_type.h"

namespace dwr {
std::vector<Pixel> BresenhamLine(const Pixel &start_pixel, const Pixel &end_pixel);
std::vector<std::vector<Pixel>> VerticalEquantLine(const Pixel &start_pixel, const Pixel &end_pixel, int segment_number, int radius);
}

#endif /* graphics_utils_h */
