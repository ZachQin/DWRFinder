//
//  raster_type.h
//  DWRFinder
//
//  Created by ZachQin on 2017/5/4.
//  Copyright © 2017年 Zach. All rights reserved.
//

#ifndef raster_type_h
#define raster_type_h

#include <math.h>

#include <functional>
#include <limits>
#include <utility>
#include <vector>

namespace dwr {

using Level = int;
using PixelDistance = double;

const PixelDistance kMaxPixelDistance = std::numeric_limits<PixelDistance>::max();

struct Pixel {
    int x;
    int y;

    Pixel() = default;

    Pixel(const Pixel &other) = default;

    Pixel(int x, int y): x(x), y(y) {}

    bool operator == (const Pixel &p) const {
        return std::tie(x, y) == std::tie(p.x, p.y);
    }

    static PixelDistance Distance(const Pixel &p1, const Pixel &p2) {
        return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
    }

    static double CosinTurnAngle(const Pixel &previous, const Pixel &current, const Pixel &next) {
        double pc_x = current.x - previous.x;
        double pc_y = current.y - previous.y;
        double cn_x = next.x - current.x;
        double cn_y = next.y - current.y;
        return (pc_x * cn_x + pc_y * cn_y) / (sqrt(pc_x * pc_x + pc_y * pc_y) * sqrt(cn_x * cn_x + cn_y * cn_y));
    }
};

const Pixel kNoPixel = {-1, -1};

using Line = std::vector<Pixel>;
using PixelPath = std::vector<Pixel>;

struct PixelInfo {
    PixelDistance actual_distance;
    PixelDistance estimated_distance;
    Level level;
    Pixel previous = kNoPixel;

    PixelInfo() = default;

    PixelInfo(const PixelInfo &) = default;

    PixelInfo(PixelDistance actual_distance, PixelDistance estimated_distance, Level level, const Pixel &previous) :
    actual_distance(actual_distance), estimated_distance(estimated_distance), level(level), previous(previous) {}
};

using PixelPair = std::pair<Pixel, Pixel>;
using PixelInfoPair = std::pair<PixelInfo, PixelInfo>;

}  // namespace dwr

namespace std {
template<>
struct hash<dwr::Pixel> {
    std::size_t operator()(const dwr::Pixel &p) const {
        return std::hash<int>()(p.x) ^ std::hash<int>()(p.y);
    }
};
}
#endif /* raster_type_h */
