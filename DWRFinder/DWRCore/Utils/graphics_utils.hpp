//
//  GraphicsUtils.hpp
//  DWRFinder
//
//  Created by ZachQin on 2017/3/16.
//  Copyright © 2017年 Zach. All rights reserved.
//

#ifndef graphics_utils_hpp
#define graphics_utils_hpp

#include <vector>
#include "raster_type.hpp"

namespace dwr {
std::vector<Pixel> BresenhamLine(const Pixel &startPoint, const Pixel &endPoint);
std::vector<std::vector<Pixel>> VerticalEquantLine(const Pixel &startPoint, const Pixel &endPoint, int segmentNumber, int radius);
}

#endif /* graphics_utils_hpp */
