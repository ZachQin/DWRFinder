//
//  main.cpp
//  DWRFinder
//
//  Created by ZachQin on 2017/3/8.
//  Copyright © 2017年 Zach. All rights reserved.
//

#include <iostream>
#include "airway_graph.hpp"
#include "dynamic_radar_airway_graph.hpp"
#include "radar_image_process.h"

#include <time.h>
#include <algorithm>

using namespace std;

int main(int argc, const char * argv[]) {
    // insert code here...
    
    dwr::DynamicRadarAirwayGraph g;
    g.LoadFromFile("/Users/ZkTsin/Desktop/bysj/AirwayGraph.ag");
    
    dwr::WorldFileInfo worldInfo("/Users/ZkTsin/Developer/GraduationDesign/qgis/test.wld");
    g.prebuild(worldInfo);
    
    // Raster start
    CGDataProviderRef provider = CGDataProviderCreateWithFilename("/Users/ZkTsin/Developer/GraduationDesign/qgis/test.png");
    CGImageRef image = CGImageCreateWithPNGDataProvider(provider, NULL, false, kCGRenderingIntentDefault);
    int width, height;
    char *mask = CreateMaskFromCGImage(image, &width, &height);
    g.UpdateBlock(mask, width, height);
//    free(mask);
    // Raster end
//    g.LogBlockAirpointSegment();

//    {
//        std::vector<AirwayPoint> path;
//        clock_t tStart = clock();
//        g.GetPath(1835, 5269, path);
//        printf("Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
//        for (auto &i: path) {
//            std::cout << i.name << "->";
//        }
//        std::cout << "🔚";
//        std::cout << std::endl;
//    }
//    {
//        
//        std::vector<AirwayPoint> path;
//        clock_t tStart = clock();
//        g.GetDynamicPath(1835, 5269, path);
//        printf("Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
//        for (auto &i: path) {
//            std::cout << i.name << "->";
//        }
//        std::cout << "🔚";
//        std::cout << std::endl;
//    }
    {
        
        clock_t tStart = clock();
        std::vector<std::shared_ptr<dwr::AirwayPoint>> path = g.GetDynamicFullPath(8071, 20631);
//        std::vector<std::shared_ptr<dwr::AirwayPoint>> path = g.GetPath(8071, 20631);
        printf("Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
        for (auto &i: path) {
            std::cout << i->name << "->";
        }
        std::cout << "🔚";
        std::cout << std::endl;
    }
    
    return 0;
}
