//
//  raster_type.h
//  DWRFinder
//
//  Created by ZachQin on 2017/5/4.
//  Copyright © 2017年 Zach. All rights reserved.
//

#ifndef raster_type_hpp
#define raster_type_hpp

#include <math.h>

typedef int Level;
typedef double PixelDistance;

const PixelDistance max_distance = std::numeric_limits<PixelDistance>::max();
const int kNoPixel = -1;

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
    
    static PixelDistance Distance(const Pixel &p1, const Pixel &p2) {
        return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
    }
};

struct NodeInfo {
    PixelDistance distance;
    PixelDistance heuristic;
    Level level;
    Pixel pixel;
    NodeInfo *previous = nullptr;
    
    NodeInfo(PixelDistance dist, PixelDistance heur, Level l, Pixel px, NodeInfo *pre): distance(dist), heuristic(heur), level(l), pixel(px), previous(pre){};
    
    bool operator > (const NodeInfo &p) const {
        if (distance == max_distance) {return true;}
        if (p.distance == max_distance) {return false;}
        return distance + heuristic > p.distance + p.heuristic;
    }
};

typedef std::pair<NodeInfo, NodeInfo> NodeInfoPair;

#endif /* raster_type_h */
