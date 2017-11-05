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

typedef int Level;
typedef double PixelDistance;

const PixelDistance max_distance = std::numeric_limits<PixelDistance>::max();

struct Pixel {
    int x;
    int y;
    
    Pixel() = default;
    Pixel(const Pixel &other) = default;
    Pixel(int x, int y): x(x), y(y) {};
    
    bool operator == (const Pixel &p) const {
        return this->x == p.x && this->y == p.y;
    }

    static PixelDistance Distance(const Pixel &p1, const Pixel &p2) {
        return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
    }
};

namespace std {
template<>
struct hash<Pixel> {
    std::size_t operator()(const Pixel& p) const {
        return std::hash<int>()(p.x) ^ std::hash<int>()(p.y);
    }
};
}

const Pixel kNoPixel = {-1, -1};

typedef std::vector<Pixel> Line;
typedef std::vector<Pixel> PixelPath;

struct PixelInfo {
    PixelDistance distance;
    PixelDistance heuristic;
    Level level;
    Pixel previous = kNoPixel;
    
    PixelInfo() = default;
    PixelInfo(const PixelInfo &) = default;
    PixelInfo(PixelDistance distance, PixelDistance heuristic, Level level, const Pixel &previous) : distance(distance), heuristic(heuristic), level(level), previous(previous) {};
    
    bool operator > (const PixelInfo &p) const {
        if (distance == max_distance) {return true;}
        if (p.distance == max_distance) {return false;}
        return distance + heuristic > p.distance + p.heuristic;
    }
};

typedef std::pair<Pixel, Pixel> PixelPair;
typedef std::pair<PixelInfo, PixelInfo> PixelInfoPair;

#endif /* raster_type_h */
