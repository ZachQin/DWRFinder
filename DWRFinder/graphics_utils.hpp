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
#include <math.h>
#include <stdlib.h>

namespace dwr {
    
typedef double Distance;

struct Pixel {
    int x;
    int y;
    
    Pixel() {};
    Pixel(int x, int y): x(x), y(y) {};
    
    bool operator < (const Pixel &p) const {
        return this->x < p.x ? true : (this->x > p.x ? false : this->y < p.y);
    }
    
    bool operator == (const Pixel &p) const {
        return this->x == p.x && this->y == p.y;
    }
    
    bool operator > (const Pixel &p) const {
        return !((*this < p) || (*this == p));
    }
    
    char getPixelValue(const char *mask, int width, int height) const {
        if (x >= 0 && x < width && y >= 0 && y < height) {
            return mask[y * width + x];
        } else {
            return 0;
        }
    };
    
    Distance DistanceTo(const Pixel &to) const {
        return sqrt((x - to.x) * (x - to.x) + (y - to.y) * (y - to.y));
    }
};

void BresenhamLine(Pixel startPoint, Pixel endPoint, std::vector<Pixel> &pixels);
void VerticalEquantLine(Pixel startPoint, Pixel endPoint, int segmentNumber, int radius, std::vector<std::vector<Pixel>> &segments);
    
}

#endif /* graphics_utils_hpp */
