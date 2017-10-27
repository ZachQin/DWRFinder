//
//  main.cc
//  DWRFinder
//
//  Created by ZachQin on 2017/3/8.
//  Copyright © 2017年 Zach. All rights reserved.
//

#include <iostream>
#include "airway_graph.h"
#include "dynamic_radar_airway_graph.h"
#include "radar_image_process.h"
#include <fstream>

#include <time.h>
#include <algorithm>

using namespace std;
void FullPath(dwr::DynamicRadarAirwayGraph &graph);
void FullPathTest(dwr::DynamicRadarAirwayGraph &graph);
void BatchTest(dwr::DynamicRadarAirwayGraph &graph, int batch_count, const std::string &path);
int PathLength(const std::vector<std::shared_ptr<dwr::Waypoint>> &path);


struct Statistics {
    double time_consuming;
    double sum_length;
    double node_count;
};

int main(int argc, const char * argv[]) {
    // insert code here...
    
    dwr::DynamicRadarAirwayGraph graph;
    graph.LoadFromFile("/Users/ZkTsin/Developer/GraduationDesign/DWRFinder/DWRFinder/Test/Resource/AirwayGraph.ag");
    
    dwr::WorldFileInfo worldInfo("/Users/ZkTsin/Developer/GraduationDesign/DWRFinder/DWRFinder/Test/Resource/WorldFile.wld");
    graph.Build(worldInfo);
    
    // Raster start
    CGDataProviderRef provider = CGDataProviderCreateWithFilename("/Users/ZkTsin/Developer/GraduationDesign/DWRFinder/DWRFinder/Test/Resource/radar.png");
    CGImageRef image = CGImageCreateWithPNGDataProvider(provider, NULL, false, kCGRenderingIntentDefault);
    int width, height;
    
    clock_t start_clock = clock();
    std::shared_ptr<const char> mask(CreateMaskFromCGImage(image, &width, &height));
    graph.UpdateBlock(mask, width, height);
    printf("Data process Time taken: %.4fms\n", (double)(clock() - start_clock) * 1000.0 / CLOCKS_PER_SEC);
    
//    FullPath(g);
    BatchTest(graph, 10000, "/Users/ZkTsin/Desktop/test_result.txt");
    return 0;
}

std::vector<dwr::WaypointIdentifier> RandomWaypointVector(dwr::AirwayGraph &graph, int size);

Statistics DoSomeStatistics(const std::function<std::vector<std::shared_ptr<dwr::Waypoint>>()> &func) {
    Statistics s;
    clock_t start_clock = clock();
    auto path = func();
    s.time_consuming = (double)(clock() - start_clock) * 1000 / CLOCKS_PER_SEC;
    s.sum_length = PathLength(path) / 1000.0;
    s.node_count = path.size();
    return s;
}

void BatchTest(dwr::DynamicRadarAirwayGraph &graph, int batch_count, const std::string &path) {
    ofstream of(path, ios::binary);
    auto random_start = RandomWaypointVector(graph, batch_count);
    auto random_end = RandomWaypointVector(graph, batch_count);
    
//    std::vector<std::tuple<bool, double, double>> results;
    for (int i = 0; i < batch_count; i++) {
        auto start = random_start[i];
        auto end = random_end[i];
        
        auto distance = dwr::Waypoint::Distance(*graph.WaypointFromIdentifier(start), *graph.WaypointFromIdentifier(end)) / 1000.0;
        auto normal = DoSomeStatistics([&](){return graph.FindPath(start, end);});
        auto dynamic = DoSomeStatistics([&](){return graph.FindDynamicPath(start, end);});
        auto full = DoSomeStatistics([&](){return graph.FindDynamicFullPath(start, end);});
        
        of << distance << ",";
        of << normal.time_consuming << "," << normal.sum_length << "," << normal.node_count;
        of << ",";
        of << dynamic.time_consuming << "," << dynamic.sum_length << "," << dynamic.node_count;
        of << ",";
        of << full.time_consuming << "," << full.sum_length << "," << full.node_count;
        of << endl;
    }
//    printf("Finish");
}



void FullPath(dwr::DynamicRadarAirwayGraph &graph) {
    // Case I
//    dwr::WaypointID start = 1644;
//    dwr::WaypointID end = 21446;
    
    // Case II
    dwr::WaypointIdentifier start = 8071;
    dwr::WaypointIdentifier end = 20631;
    
    clock_t tStart = clock();
    std::vector<std::shared_ptr<dwr::Waypoint>> path = graph.FindDynamicFullPath(start, end);
    printf("Time taken: %.4fms\n", (double)(clock() - tStart) * 1000 / CLOCKS_PER_SEC);
    for (auto &i: path) {
        cout << i->name << "->";
    }
    cout << "end" << endl;
    cout << "Length:" << PathLength(path) << endl;
}

std::vector<dwr::WaypointIdentifier> RandomWaypointVector(dwr::AirwayGraph &graph, int count) {
    auto all_vector = graph.AllWaypointIdentifiers();
    assert(all_vector.size() > 0);
    std::vector<dwr::WaypointIdentifier> random_vector;
    for (int i = 0; i < count; i++) {
        dwr::WaypointIdentifier random_waypoint_identifier = 0;
        std::shared_ptr<dwr::Waypoint> randomWaypoint = nullptr;
        do {
            int random_index = rand() % all_vector.size();
            random_waypoint_identifier = all_vector[random_index];
            randomWaypoint = graph.WaypointFromIdentifier(random_waypoint_identifier);
        } while (randomWaypoint->neibors.empty());
        random_vector.push_back(random_waypoint_identifier);
    }
    return random_vector;
}

int PathLength(const std::vector<std::shared_ptr<dwr::Waypoint>> &path) {
    if (path.empty()) {
        return 0;
    }
    dwr::GeoDistance sum = 0;
    auto bp = path[0];
    for (int i = 1; i < path.size(); i++) {
        auto fp = path[i];
        sum += dwr::Waypoint::Distance(*bp, *fp);
        bp = fp;
    }
    return sum;
}

void FullPathTest(dwr::DynamicRadarAirwayGraph &graph) {
    std::string result_string;
    clock_t start_clock = clock();
    std::vector<std::shared_ptr<dwr::Waypoint>> path = graph.FindDynamicFullPath(8071, 20631);
    printf("Time taken: %.2fms\n", (double)(clock() - start_clock) * 1000.0 / CLOCKS_PER_SEC);
    for (auto &i: path) {
        result_string.append(i->name);
        result_string.append("->");
    }
    cout << result_string << endl;
    string ground_string = "P130->XIVEP->ANPIG->EGEBI->长治->P106->P279->济源->洛阳->P320->P339->南阳->襄阳->112.18E32.01N->112.18E32.01N->P38->临澧->常德->老粮仓->111.52E27.71N->111.42E27.66N->P347->P378->P246->110.70E25.99N->110.51E25.78N->ONEMI->大榕江->奇峰岭->二塘->MUBEL->高要->P50->";
    if (result_string == ground_string) {
        cout << "Pass!" << endl;
    } else {
        cout << "Not Pass!" << endl;
    }
}
