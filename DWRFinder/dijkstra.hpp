//
//  dijkstra.hpp
//  DWRFinder
//
//  Created by ZachQin on 2017/3/1.
//  Copyright © 2017年 Zach. All rights reserved.
//

#ifndef dijkstra_hpp
#define dijkstra_hpp

#include <stdio.h>
#include <vector>
#include <list>
#include <functional>
#include "graph.h"


void DijkstraComputePaths(Vertex source, Vertex dest,
                          const adjacency_list_t &adjacency_list,
                          std::vector<Weight> &min_distance,
                          std::vector<Vertex> &previous,
                          const std::function<bool(Edge, std::vector<Vertex> &)> &canSearch = [](Edge edge, std::vector<Vertex> &previes){return true;});
std::list<Vertex> DijkstraGetShortestPath(Vertex vertex, const std::vector<Vertex> &previous);

#endif /* dijkstra_hpp */
