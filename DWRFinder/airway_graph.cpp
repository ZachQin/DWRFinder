//
//  AirwayGraph.cpp
//  DWR
//
//  Created by ZachQin on 2017/3/3.
//  Copyright © 2017年 Zach. All rights reserved.
//

#include "airway_graph.hpp"
#include "dijkstra.hpp"
#include <assert.h>
#include <fstream>

AirwayGraph::AirwayGraph(const char *path) {
    this->LoadFromFile(path);
}

void AirwayGraph::AddAirwayPoint(AirwayPointID identity, std::string name, double x, double y, double lon, double lat) {
    if (airwayPointMap_.find(identity) != airwayPointMap_.end()) {
        return;
    }
    AirwayPoint point(identity, name, x, y, lon, lat);
    airwayPointMap_.insert(std::make_pair(identity, airwayPointVector_.size()));
    airwayPointVector_.push_back(point);
    adjacencyList_.push_back(std::vector<Neighbor>());
}

void AirwayGraph::AddAirwaySegment(AirwayPointID identity1, AirwayPointID identity2) {
    Vertex vertex1 = airwayPointMap_[identity1];
    Vertex vertex2 = airwayPointMap_[identity2];
    double distance = airwayPointVector_[vertex1].distance(airwayPointVector_[vertex2]);
    
    Neighbor nb1(vertex2, distance);
    Neighbor nb2(vertex1, distance);
    adjacencyList_[vertex1].push_back(nb1);
    adjacencyList_[vertex2].push_back(nb2);
}

void AirwayGraph::GetPath(AirwayPointID sourceIdentity, AirwayPointID destinIdentity, std::vector<AirwayPoint> &path, const std::function<bool(Edge, std::vector<Vertex> &)> &canSearch) {
    Vertex sourceVertex = airwayPointMap_[sourceIdentity];
    Vertex destVertex = airwayPointMap_[destinIdentity];
    std::vector<Weight> min_distance;
    std::vector<Vertex> previous;
    DijkstraComputePaths(sourceVertex, destVertex, adjacencyList_, min_distance, previous, canSearch);
    std::list<Vertex> rawPath = DijkstraGetShortestPath(destVertex, previous);
    
    path.clear();
    for (auto &vertex: rawPath) {
        path.push_back(airwayPointVector_[vertex]);
    }
}

bool AirwayGraph::SaveToFile(std::string path) {
    uint32_t n = (uint32_t)airwayPointMap_.size();
    uint32_t n2 = (uint32_t)airwayPointVector_.size();
    uint32_t n3 = (uint32_t)adjacencyList_.size();
    assert(n == n2 && n == n3);
    
    std::ofstream of(path, std::ios::binary);
    if (!of.is_open()) {
        return false;
    }
    of.write((char *)&n, sizeof(uint32_t));
    for (auto &c: airwayPointMap_) {
        uint32_t identity = (uint32_t)c.first;
        uint32_t vertex = (uint32_t)c.second;
        of.write((char *)&identity, sizeof(uint32_t));
        of.write((char *)&vertex, sizeof(uint32_t));
    }
    of.write((char *)&n2, sizeof(uint32_t));
    for (auto &c: airwayPointVector_) {
        AirwayPointID airwaypointID = (AirwayPointID)c.airwayPointID;
        of.write((char *)&airwaypointID, sizeof(AirwayPointID));
        uint32_t nameSize = (uint32_t)c.name.size();
        of.write((char *)&nameSize, sizeof(uint32_t));
        of.write(c.name.c_str(), nameSize);
        of.write((char *)&c.x, sizeof(double));
        of.write((char *)&c.y, sizeof(double));
        of.write((char *)&c.longitude, sizeof(double));
        of.write((char *)&c.latitude, sizeof(double));
    }
    of.write((char *)&n3, sizeof(uint32_t));
    for (auto &c: adjacencyList_) {
        uint32_t nn = (uint32_t)c.size();
        of.write((char *)&nn, sizeof(uint32_t));
        for (auto &cc: c) {
            of.write((char *)&cc, sizeof(Neighbor));
        }
    }
    of.flush();
    return true;
}

bool AirwayGraph::LoadFromFile(std::string path) {
    std::ifstream inf(path, std::ios::binary);
    if (!inf.is_open()) {
        return false;
    }
    uint32_t n;
    inf.read((char *)&n, sizeof(uint32_t));
    for (size_t i = 0; i < n; i++) {
        AirwayPointID identity;
        inf.read((char *)&identity, sizeof(AirwayPointID));
        Vertex v;
        inf.read((char *)&v, sizeof(Vertex));
        airwayPointMap_[identity] = v;
    }
    uint32_t n2;
    inf.read((char *)&n2, sizeof(n2));
    for (uint32_t i = 0; i < n2; i++) {
        AirwayPoint point;
        inf.read((char *)&point.airwayPointID, sizeof(uint32_t));
        uint32_t nameSize;
        inf.read((char *)&nameSize, sizeof(uint32_t));
        char *nameBuffer = new char[nameSize + 1];
        nameBuffer[nameSize] = 0;
        inf.read(nameBuffer, nameSize);
        point.name = std::string(nameBuffer);
        delete[] nameBuffer;
        inf.read((char *)&point.x, sizeof(double));
        inf.read((char *)&point.y, sizeof(double));
        inf.read((char *)&point.longitude, sizeof(double));
        inf.read((char *)&point.latitude, sizeof(double));
        point.userWaypoint = false;
        airwayPointVector_.push_back(point);
    }
    uint32_t n3;
    inf.read((char *)&n3, sizeof(uint32_t));
    for (uint32_t i = 0; i < n3; i++) {
        uint32_t nn;
        inf.read((char *)&nn, sizeof(uint32_t));
        std::vector<Neighbor> vnb;
        for (uint32_t j = 0; j < nn; j++) {
            Neighbor nb;
            inf.read((char *)&nb, sizeof(Neighbor));
            vnb.push_back(nb);
        }
        adjacencyList_.push_back(vnb);
    }
    assert(n == n2 && n == n3);
    return true;
}

void AirwayGraph::ForEach(std::function<void (AirwayPoint, AirwayPoint, double)> &traverseFunction) {
    for (Vertex startVertex = 0; startVertex < adjacencyList_.size(); startVertex++) {
        auto neighbors = adjacencyList_[startVertex];
        for (int index = 0; index < neighbors.size(); index++) {
            auto neighbor = neighbors[index];
            auto endVertex = neighbor.target;
            auto startAirwayPoint = airwayPointVector_[startVertex];
            auto endAirwayPoint = airwayPointVector_[endVertex];
            auto length = neighbor.weight;
            traverseFunction(startAirwayPoint, endAirwayPoint, length);
        }
    }
}

AirwayPoint AirwayGraph:: AirwayPointFromID(AirwayPointID identity) {
    auto iterator = airwayPointMap_.find(identity);
    if (iterator != airwayPointMap_.end()) {
        return airwayPointVector_[iterator->second];
    } else {
        return AirwayPoint();
    }
}
