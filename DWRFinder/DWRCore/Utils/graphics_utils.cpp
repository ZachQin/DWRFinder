//
//  GraphicsUtils.cpp
//  DWRFinder
//
//  Created by ZachQin on 2017/3/16.
//  Copyright © 2017年 Zach. All rights reserved.
//

#include "graphics_utils.hpp"
#include <stdlib.h>

namespace dwr {
std::vector<Pixel> BresenhamLine(const Pixel &startPoint, const Pixel &endPoint) {
    std::vector<Pixel> result;
    int x0 = startPoint.x, x1 = endPoint.x;
    int y0 = startPoint.y, y1 = endPoint.y;
    bool steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep) {
        std::swap(x0, y0);
        std::swap(x1, y1);
    }
    bool reverse = x0 > x1;
    if (reverse) {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }
    int deltax = x1 - x0;
    int deltay = abs(y1 - y0);
    int error = deltax / 2;
    int yy = y0;
    int ystep = y0 < y1 ? 1 : -1;
    for (int xx = x0; xx <= x1; xx++) {
        if (steep) {
            result.push_back(Pixel(yy, xx));
        } else {
            result.push_back(Pixel(xx, yy));
        }
        error -= deltay;
        if (error < 0) {
            yy += ystep;
            error += deltax;
        }
    }
    if (reverse) {
        std::reverse(result.begin(), result.end());
    }
    return result;
}

std::vector<std::vector<Pixel>> VerticalEquantLine(const Pixel &startPoint, const Pixel &endPoint, int segmentNumber, int radius) {
    std::vector<std::vector<Pixel>> result;
    int x0 = startPoint.x, x1 = endPoint.x;
    int y0 = startPoint.y, y1 = endPoint.y;
    bool steep = abs(y1 - y0) < abs(x1 - x0);
    if (steep) {
        std::swap(x0, y0);
        std::swap(x1, y1);
    }
    double k = - (double)(x1 - x0) / (y1 - y0);
    
    int dx = round(sqrt(1.0 / (k * k + 1)) * radius);
    int dy = round(k * dx);
    
    double segmentDx = (double)(x1 - x0) / segmentNumber;
    double segmentDy = (double)(y1 - y0) / segmentNumber;
    
    for (int i = 1; i < segmentNumber; i++) {
        Pixel point(x0 + segmentDx * i, y0 + segmentDy * i);
        Pixel verticalStartPoint = steep ? Pixel(point.y - dy, point.x - dx) : Pixel(point.x - dx, point.y - dy);
        Pixel verticalEndPoint = steep ? Pixel(point.y + dy, point.x + dx) : Pixel(point.x + dx, point.y + dy);
        std::vector<Pixel> verticalLine = BresenhamLine(verticalStartPoint, verticalEndPoint);
        result.push_back(verticalLine);
    }
    return result;
};
    
}
