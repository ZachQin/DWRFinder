//
//  GraphicsUtils.hpp
//  DWRFinder
//
//  Created by ZachQin on 2017/3/16.
//  Copyright © 2017年 Zach. All rights reserved.
//

#ifndef graphics_utils_hpp
#define graphics_utils_hpp

#include <stdio.h>
#include <vector>
#include "raster_type.hpp"

namespace dwr {
void BresenhamLine(const Pixel &startPoint, const Pixel &endPoint, std::vector<Pixel> &pixels);
void VerticalEquantLine(const Pixel &startPoint, const Pixel &endPoint, int segmentNumber, int radius, std::vector<std::vector<Pixel>> &segments);
}

#endif /* graphics_utils_hpp */
