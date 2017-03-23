//
//  dijkstra.cpp
//  DWRFinder
//
//  Created by ZachQin on 2017/3/1.
//  Copyright © 2017年 Zach. All rights reserved.
//

#include "dijkstra.hpp"
#include <queue>

const Weight max_weight = std::numeric_limits<double>::infinity();

typedef std::pair<Weight, Vertex> weight_vertex_pair_t;

void DijkstraComputePaths(Vertex source, Vertex dest,
                          const adjacency_list_t &adjacency_list,
                          std::vector<Weight> &min_distance,
                          std::vector<Vertex> &previous,
                          const std::function<bool(Edge, std::vector<Vertex> &)> &canSearch) {
    int n = (int)adjacency_list.size();
    min_distance.clear();
    min_distance.resize(n, max_weight);
    min_distance[source] = 0;
    previous.clear();
    previous.resize(n, -1);
    // we use greater instead of less to turn max-heap into min-heap
    std::priority_queue<weight_vertex_pair_t,
    std::vector<weight_vertex_pair_t>,
    std::greater<weight_vertex_pair_t> > vertex_queue;
    vertex_queue.push(std::make_pair(min_distance[source], source));
    
    while (!vertex_queue.empty())
    {
        Weight dist = vertex_queue.top().first;
        Vertex u = vertex_queue.top().second;
        vertex_queue.pop();
        
        if (u == dest) {
            return;
        }
        
        // Because we leave old copies of the vertex in the priority queue
        // (with outdated higher distances), we need to ignore it when we come
        // across it again, by checking its distance against the minimum distance
        if (dist > min_distance[u])
            continue;
        
        // Visit each edge exiting u
        for (auto &neighbor: adjacency_list[u])
        {
            Vertex v = neighbor.target;
            if (!canSearch(Edge(u, v), previous)) {
                continue;
            }
            Weight weight = neighbor.weight;
            Weight distance_through_u = dist + weight;
            if (distance_through_u < min_distance[v]) {
                min_distance[v] = distance_through_u;
                previous[v] = u;
                vertex_queue.push(std::make_pair(min_distance[v], v));
            }
        }
    }
}

std::list<Vertex> DijkstraGetShortestPath(Vertex vertex, const std::vector<Vertex> &previous)
{
    std::list<Vertex> path;
    for ( ; vertex != -1; vertex = previous[vertex])
        path.push_front(vertex);
    return path;
}
