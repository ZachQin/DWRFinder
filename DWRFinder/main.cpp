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
void FullPath(dwr::DynamicRadarAirwayGraph &g);
void FullPathTest(dwr::DynamicRadarAirwayGraph &g);

int main(int argc, const char * argv[]) {
    // insert code here...
    
    dwr::DynamicRadarAirwayGraph g;
    g.LoadFromFile("/Users/ZkTsin/Desktop/bysj/AirwayGraph.ag");
    
    dwr::WorldFileInfo worldInfo("/Users/ZkTsin/Developer/GraduationDesign/qgis/test.wld");
    g.prebuild(worldInfo);
    
    // Raster start
    CGDataProviderRef provider = CGDataProviderCreateWithFilename("/Users/ZkTsin/Desktop/MyPaper/未命名文件夹/radar_1.png");
    CGImageRef image = CGImageCreateWithPNGDataProvider(provider, NULL, false, kCGRenderingIntentDefault);
    int width, height;
    
    clock_t tStart = clock();
    std::shared_ptr<const char> mask(CreateMaskFromCGImage(image, &width, &height));
    g.UpdateBlock(mask, width, height);
    printf("Data process Time taken: %.4fms\n", (double)(clock() - tStart) * 1000.0 / CLOCKS_PER_SEC);
    
    FullPathTest(g);
    return 0;
}

void FullPath(dwr::DynamicRadarAirwayGraph &g) {
    // Case I
    dwr::WaypointID start = 1644;
    dwr::WaypointID end = 21446;
    
    // Case II
//    dwr::WaypointID start = 8071;
//    dwr::WaypointID end = 20631;
    
    clock_t tStart = clock();
    std::vector<std::shared_ptr<dwr::Waypoint>> path = g.GetDynamicFullPath(start, end);
    printf("Time taken: %.4fms\n", (double)(clock() - tStart) * 1000 /CLOCKS_PER_SEC);
    for (auto &i: path) {
        cout << i->name << "->";
    }
    cout << "end" << endl;
}

void FullPathTest(dwr::DynamicRadarAirwayGraph &g) {
    std::string resultStr;
    clock_t tStart = clock();
    std::vector<std::shared_ptr<dwr::Waypoint>> path = g.GetDynamicFullPath(8071, 20631);
    printf("Time taken: %.2fms\n", (double)(clock() - tStart) * 1000.0 / CLOCKS_PER_SEC);
    for (auto &i: path) {
        resultStr.append(i->name);
        resultStr.append("->");
    }
//    cout << resultStr << endl;
    string groundStr = "P130->XIVEP->ANPIG->EGEBI->长治->P106->P279->济源->洛阳->P320->P339->南阳->襄阳->112.18E32.01N->112.18E32.01N->P38->临澧->常德->老粮仓->111.52E27.71N->111.42E27.66N->P347->P378->P246->110.70E25.99N->110.51E25.78N->ONEMI->大榕江->奇峰岭->二塘->MUBEL->高要->P50->";
    if (resultStr == groundStr) {
        cout << "Pass!" << endl;
    } else {
        cout << "Not Pass!" << endl;
    }
}
