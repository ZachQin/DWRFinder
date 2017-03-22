//
//  graph.h
//  DWR
//
//  Created by ZachQin on 2017/3/3.
//  Copyright © 2017年 Zach. All rights reserved.
//

#ifndef graph_h
#define graph_h

#include <vector>

typedef uint32_t Vertex;
typedef double Weight;
typedef std::pair<Vertex, Vertex> Edge;

//struct Edge {
//    Vertex small;
//    Vertex big;
//    
//    Edge(Vertex v1, Vertex v2) {
//        small = v1;
//        big = v2;
//        if (small > big) {std::swap(small, big);}
//    };
//    
//    bool operator < (const Edge &p) const {
//        return this->small < p.small ? true : (this->small > p.small ? false : this->big < p.big);
//    }
//};

struct Neighbor {
    Vertex target;
    Weight weight;
    Neighbor() {};
    Neighbor(Vertex arg_target, Weight arg_weight)
    : target(arg_target), weight(arg_weight) { }
}__attribute__ ((aligned (8)));;

typedef std::vector<std::vector<Neighbor> > adjacency_list_t;
typedef std::pair<Weight, Vertex> weight_vertex_pair_t;

#endif /* graph_h */
