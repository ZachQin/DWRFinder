//
//  GraphicsUtils.cc
//  DWRFinder
//
//  Created by ZachQin on 2017/3/16.
//  Copyright © 2017年 Zach. All rights reserved.
//

#include "graphics_utils.h"

namespace dwr {
Line BresenhamLine(const Pixel &start_pixel, const Pixel &end_pixel) {
    Line result;
    int x0 = start_pixel.x, x1 = end_pixel.x;
    int y0 = start_pixel.y, y1 = end_pixel.y;
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
    int delta_x = x1 - x0;
    int delta_y = abs(y1 - y0);
    int error = delta_x / 2;
    int yy = y0;
    int ystep = y0 < y1 ? 1 : -1;
    for (int xx = x0; xx <= x1; xx++) {
        if (steep) {
            result.push_back(Pixel(yy, xx));
        } else {
            result.push_back(Pixel(xx, yy));
        }
        error -= delta_y;
        if (error < 0) {
            yy += ystep;
            error += delta_x;
        }
    }
    if (reverse) {
        std::reverse(result.begin(), result.end());
    }
    return result;
}

std::vector<Line> VerticalEquantLine(const Pixel &start_pixel, const Pixel &end_pixel, int segment_number, int radius) {
    std::vector<Line> result;
    int x0 = start_pixel.x, x1 = end_pixel.x;
    int y0 = start_pixel.y, y1 = end_pixel.y;
    bool steep = abs(y1 - y0) < abs(x1 - x0);
    if (steep) {
        std::swap(x0, y0);
        std::swap(x1, y1);
    }
    double k = -static_cast<double>(x1 - x0) / (y1 - y0);
    
    int dx = round(sqrt(1.0 / (k * k + 1)) * radius);
    int dy = round(k * dx);
    
    double segment_dx = static_cast<double>(x1 - x0) / segment_number;
    double segment_dy = static_cast<double>(y1 - y0) / segment_number;
    
    for (int i = 1; i < segment_number; i++) {
        Pixel point(x0 + segment_dx * i, y0 + segment_dy * i);
        Pixel vertical_start_point = steep ? Pixel(point.y - dy, point.x - dx) : Pixel(point.x - dx, point.y - dy);
        Pixel vertical_end_point = steep ? Pixel(point.y + dy, point.x + dx) : Pixel(point.x + dx, point.y + dy);
        Line vertical_line = BresenhamLine(vertical_start_point, vertical_end_point);
        result.push_back(std::move(vertical_line));
    }
    return result;
};
    
}
