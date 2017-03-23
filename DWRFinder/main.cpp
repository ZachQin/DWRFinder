//
//  main.cpp
//  DWRFinder
//
//  Created by ZachQin on 2017/3/8.
//  Copyright © 2017年 Zach. All rights reserved.
//

#include <iostream>
#include "airway_graph.hpp"
#include <unordered_map>
#include "dynamic_radar_airway_graph.hpp"

#include "radar_image_process.h"

#include <time.h>

using namespace std;

int main(int argc, const char * argv[]) {
    // insert code here...
    
    DynamicRadarAirwayGraph g;
    g.LoadFromFile("/Users/ZkTsin/Desktop/bysj/AirwayGraph.ag");
    
    WorldFileInfo worldInfo("/Users/ZkTsin/Developer/GraduationDesign/qgis/test.wld");
    g.prebuild(worldInfo);
    
    // Raster start
    CGDataProviderRef provider = CGDataProviderCreateWithFilename("/Users/ZkTsin/Developer/GraduationDesign/qgis/test.png");
    CGImageRef image = CGImageCreateWithPNGDataProvider(provider, NULL, false, kCGRenderingIntentDefault);
    size_t width, height;
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
        std::vector<AirwayPoint> path;
        clock_t tStart = clock();
        g.GetDynamicFullPath(21690, 1436, path);
        printf("Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
        for (auto &i: path) {
            std::cout << i.name << "->";
        }
        std::cout << "🔚";
        std::cout << std::endl;
    }
    
    return 0;
}
