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
#include <algorithm>

using namespace std;
void FullPath(const dwr::DynamicRadarAirwayGraph &graph);
void FullPathTest(const dwr::DynamicRadarAirwayGraph &graph);
void BatchTest(const dwr::DynamicRadarAirwayGraph &graph, int batch_count, const string &path);
int PathLength(const dwr::WaypointPath &path);
string PathDescription(const dwr::WaypointPath &path);
vector<dwr::WaypointIdentifier> RandomWaypointVector(const dwr::AirwayGraph &graph, int size);
void GenerateBatchTestGround(const dwr::DynamicRadarAirwayGraph &graph, int batch_count, const string &path);
void BatchTestWithGround(const dwr::DynamicRadarAirwayGraph &graph, const string &path);

struct Statistics {
    double time_consuming;
    double sum_length;
    double node_count;
    dwr::WaypointPath path;
};

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
    shared_ptr<const char> mask(CreateMaskFromCGImage(image, &width, &height));
    graph.UpdateBlock(mask, width, height);
    printf("Data process Time taken: %.4fms\n", (double)(clock() - start_clock) * 1000.0 / CLOCKS_PER_SEC);

//    GenerateBatchTestGround(graph, 200, "/Users/ZkTsin/Desktop/temp_test/test.txt");
    BatchTestWithGround(graph, "/Users/ZkTsin/Developer/GraduationDesign/DWRFinder/DWRFinder/Test/Resource/batch_test.txt");
//    FullPathTest(graph);
//    FullPath(graph);
//    BatchTest(graph, 10000, "/Users/ZkTsin/Desktop/test_result.txt");
    
    return 0;
}

Statistics MeasurePath(const function<dwr::WaypointPath()> &func) {
    Statistics s;
    clock_t start_clock = clock();
    s.path = func();
    s.time_consuming = (double)(clock() - start_clock) * 1000 / CLOCKS_PER_SEC;
    s.sum_length = PathLength(s.path) / 1000.0;
    s.node_count = s.path.size();
    return s;
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
        string path_description = dwr::PathDescription(stat.path);
        of << start << "," << end << "," << path_description << endl;
    }
    of << time_consuming << endl;
}

void BatchTestWithGround(const dwr::DynamicRadarAirwayGraph &graph, const string &path) {
    ifstream inf(path, ios::binary);
    string count_line;
    getline(inf, count_line);
    double time_consuming = 0;
    int batch_count = stoi(count_line);
    int failure_index = -1;
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
        string path_description = item;
        Statistics stat = MeasurePath([&](){return graph.FindDynamicFullPath(start, end);});
        time_consuming += stat.time_consuming;
        if (path_description == dwr::PathDescription(stat.path)) {
            cout << "Test index " << i << ":" << "Pass" << endl;
        } else {
            failure_index = i;
            break;
        }
    }
    if (failure_index == -1) {
        cout << "All Pass, Time-consuming: " << time_consuming << "ms" << endl;
    } else {
        cout << "Fail at index" << failure_index << endl;
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
        of << normal.time_consuming << "," << normal.sum_length << "," << normal.node_count;
        of << ",";
        of << dynamic.time_consuming << "," << dynamic.sum_length << "," << dynamic.node_count;
        of << ",";
        of << full.time_consuming << "," << full.sum_length << "," << full.node_count;
        of << endl;
    }
}



void FullPath(const dwr::DynamicRadarAirwayGraph &graph) {
    // Case I
//    dwr::WaypointID start = 1644;
//    dwr::WaypointID end = 21446;
    
    // Case II
    dwr::WaypointIdentifier start = 8071;
    dwr::WaypointIdentifier end = 20631;
    
    clock_t tStart = clock();
    vector<shared_ptr<dwr::Waypoint>> path = graph.FindDynamicFullPath(start, end);
    printf("Time taken: %.4fms\n", (double)(clock() - tStart) * 1000 / CLOCKS_PER_SEC);
    for (auto &i: path) {
        cout << i->name << "->";
    }
    cout << "end" << endl;
    cout << "Length:" << PathLength(path) << endl;
}

vector<dwr::WaypointIdentifier> RandomWaypointVector(const dwr::AirwayGraph &graph, int count) {
    auto all_vector = graph.AllWaypointIdentifiers();
    assert(all_vector.size() > 0);
    vector<dwr::WaypointIdentifier> random_vector;
    for (int i = 0; i < count; i++) {
        dwr::WaypointIdentifier random_waypoint_identifier = 0;
        shared_ptr<dwr::Waypoint> random_waypoint = nullptr;
        do {
            int random_index = rand() % all_vector.size();
            random_waypoint_identifier = all_vector[random_index];
            random_waypoint = graph.WaypointFromIdentifier(random_waypoint_identifier);
        } while (random_waypoint->neibors.empty());
        random_vector.push_back(random_waypoint_identifier);
    }
    return random_vector;
}

int PathLength(const dwr::WaypointPath &path) {
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

void FullPathTest(const dwr::DynamicRadarAirwayGraph &graph) {
    Statistics statistics = MeasurePath([&graph](){
        return graph.FindDynamicFullPath(8071, 20631);
    });
    cout << "Time taken: " << statistics.time_consuming << "ms" << endl;
    auto path_description = dwr::PathDescription(statistics.path);
    cout << path_description << endl;
    string ground_description = "P130->P430->OKVUM->NUGLA->觅子->三原->烟庄->宁陕->P40->P322->P374->SULEP->合流水->统景场->XOLAL->UNRIX->BONSA->UPKUS->P440->P441->MEMAG->IGLIT->LIKRI->XINSU->KAGRA->ESNIB->ELKAL->永福->110.23E24.98N->110.33E24.89N->MUBEL->高要->P50";
    if (path_description == ground_description) {
        cout << "Pass!" << endl;
    } else {
        cout << "Not Pass!" << endl;
    }
}
