//
//  main.cc
//  DWRFinder
//
//  Created by ZachQin on 2017/3/8.
//  Copyright © 2017年 Zach. All rights reserved.
//

#include <iostream>
#include <fstream>
#include <sstream>

#include "airway_graph.h"
#include "dynamic_radar_airway_graph.h"
#include "radar_image_process.h"

#include <time.h>

using namespace std;

struct Statistics {
    double time_consuming;
    double node_count;
    double sum_length;
    double sum_turning;
    dwr::WaypointPath path;
};

Statistics MeasurePath(const function<dwr::WaypointPath()> &func) {
    Statistics s;
    clock_t start_clock = clock();
    s.path = func();
    s.time_consuming = (double)(clock() - start_clock) * 1000 / CLOCKS_PER_SEC;
    s.node_count = s.path.GetSize();
    s.sum_length = s.path.lengths.empty() ? 0 : s.path.lengths.back();
    s.sum_turning = s.path.GetSumTurn();
    return s;
}

vector<dwr::WaypointIdentifier> RandomWaypointVector(const dwr::AirwayGraph &graph, int count) {
    auto all_vector = graph.AllWaypointIdentifiers();
    assert(all_vector.size() > 0);
    vector<dwr::WaypointIdentifier> random_vector;
    for (int i = 0; i < count; i++) {
        dwr::WaypointIdentifier random_waypoint_identifier = 0;
        dwr::WaypointPtr random_waypoint = nullptr;
        do {
            int random_index = rand() % all_vector.size();
            random_waypoint_identifier = all_vector[random_index];
            random_waypoint = graph.WaypointFromIdentifier(random_waypoint_identifier);
        } while (random_waypoint->neibors.empty());
        random_vector.push_back(random_waypoint_identifier);
    }
    return random_vector;
}

void GenerateBatchTestGround(const dwr::DynamicRadarAirwayGraph &graph, int batch_count, const string &path) {
    vector<dwr::WaypointIdentifier> random_start = RandomWaypointVector(graph, batch_count);
    vector<dwr::WaypointIdentifier> random_end = RandomWaypointVector(graph, batch_count);
    
    ofstream of(path, ios::binary);
    of << batch_count << endl;
    double time_consuming = 0;
    for (int i = 0; i < batch_count; i++) {
        dwr::WaypointIdentifier start = random_start[i];
        dwr::WaypointIdentifier end = random_end[i];
        Statistics stat = MeasurePath([&](){return graph.FindDynamicFullPath(start, end);});
        time_consuming += stat.time_consuming;
        of << start << "," << end << "," << stat.path.ToString() << endl;
    }
    of << time_consuming << endl;
}

void BatchTestWithGround(const dwr::DynamicRadarAirwayGraph &graph, const string &path) {
    ifstream inf(path, ios::binary);
    string count_line;
    getline(inf, count_line);
    double time_consuming = 0;
    int batch_count = stoi(count_line);
    int pass_count = 0;
    for (int i = 0; i < batch_count; i++) {
        string line;
        getline(inf, line);
        stringstream ss(line);
        string item;
        getline(ss, item, ',');
        dwr::WaypointIdentifier start = stoi(item);
        getline(ss, item, ',');
        dwr::WaypointIdentifier end = stoi(item);
        getline(ss, item, ',');
        string ground_path_description = item;
        Statistics stat = MeasurePath([&](){return graph.FindDynamicFullPath(start, end);});
        time_consuming += stat.time_consuming;
        string path_description = stat.path.ToString();
        if (path_description == ground_path_description) {
            pass_count++;
            cout << "Test index " << i << ":" << "Pass" << endl;
        } else {
            cout << "Test index " << i << ":" << "Failed!" << endl;
            cout << "origin: " << start << " destination: " << end << endl;
            cout << "Path" << endl;
            cout << path_description << endl;
            cout << "Groud Path:" << endl;
            cout << ground_path_description << endl;
        }
    }
    if (pass_count == batch_count) {
        cout << "All Pass, Time-consuming: " << time_consuming << "ms" << endl;
    } else {
        cout << "Pass " << pass_count << " in " << batch_count << endl;
        cout << "Time-consuming: " << time_consuming << "ms" << endl;
    }
}

void BatchTest(const dwr::DynamicRadarAirwayGraph &graph, int batch_count, const string &path) {
    ofstream of(path, ios::binary);
    auto random_start = RandomWaypointVector(graph, batch_count);
    auto random_end = RandomWaypointVector(graph, batch_count);
    
    for (int i = 0; i < batch_count; i++) {
        auto start = random_start[i];
        auto end = random_end[i];
        
        auto distance = dwr::Waypoint::Distance(*graph.WaypointFromIdentifier(start), *graph.WaypointFromIdentifier(end)) / 1000.0;
        auto normal = MeasurePath([&](){return graph.FindPath(start, end);});
        auto dynamic = MeasurePath([&](){return graph.FindDynamicPath(start, end);});
        auto full = MeasurePath([&](){return graph.FindDynamicFullPath(start, end);});
        
        of << distance << ",";
        of << normal.time_consuming << "," << normal.path.lengths.back() << "," << normal.node_count;
        of << ",";
        of << dynamic.time_consuming << "," << dynamic.path.lengths.back() << "," << dynamic.node_count;
        of << ",";
        of << full.time_consuming << "," << full.path.lengths.back() << "," << full.node_count;
        of << endl;
    }
}



void FullPath(const dwr::DynamicRadarAirwayGraph &graph) {
    dwr::WaypointIdentifier start = 8071;
    dwr::WaypointIdentifier end = 20631;
    
    auto paths = graph.FindKDynamicFullPath(start, end, 10);
    int index = 1;
    for (auto &path : paths) {
        cout << index++ << ":" << endl;
        cout << path.ToString() << endl;
    }
}

int main(int argc, const char * argv[]) {
    dwr::DynamicRadarAirwayGraph graph;
    graph.LoadFromFile("/Users/ZkTsin/Developer/GraduationDesign/DWRFinder/DWRFinder/Test/Resource/AirwayGraph.ag");
    
    dwr::WorldFileInfo worldInfo("/Users/ZkTsin/Developer/GraduationDesign/DWRFinder/DWRFinder/Test/Resource/WorldFile.wld");
    graph.Build(worldInfo);
    
    // Raster start
    CGDataProviderRef provider = CGDataProviderCreateWithFilename("/Users/ZkTsin/Developer/GraduationDesign/DWRFinder/DWRFinder/Test/Resource/radar.png");
    CGImageRef image = CGImageCreateWithPNGDataProvider(provider, NULL, false, kCGRenderingIntentDefault);
    int width, height;
    
    clock_t start_clock = clock();
    char *mask = CreateMaskFromCGImage(image, &width, &height);
    graph.UpdateBlock(mask, width, height);
    printf("Data process Time taken: %.4fms\n", (double)(clock() - start_clock) * 1000.0 / CLOCKS_PER_SEC);
    
//    GenerateBatchTestGround(graph, 200, "/Users/ZkTsin/Desktop/temp_test/test.txt");
    BatchTestWithGround(graph, "/Users/ZkTsin/Developer/GraduationDesign/DWRFinder/DWRFinder/Test/Resource/batch_test.txt");
    //    FullPath(graph);
    //    FullPath(graph);
    //    BatchTest(graph, 10000, "/Users/ZkTsin/Desktop/test_result.txt");
    return 0;
}
