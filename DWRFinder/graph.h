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
#include <set>

namespace dwr {

typedef int32_t Vertex;
typedef double Weight;
typedef std::pair<Vertex, Vertex> Edge;

struct UndirectedEdge {
    Vertex small;
    Vertex big;
    
    UndirectedEdge(Edge &edge) {
        small = edge.first;
        big = edge.second;
        if (small > big) {std::swap(small, big);}
    }
    
    UndirectedEdge(Vertex v1, Vertex v2) {
        small = v1;
        big = v2;
        if (small > big) {std::swap(small, big);}
    };
    
    bool operator < (const UndirectedEdge &p) const {
        return this->small < p.small ? true : (this->small > p.small ? false : this->big < p.big);
    }
};

struct Neighbor {
    Vertex target;
    Weight weight;
    Neighbor() {};
    Neighbor(Vertex arg_target, Weight arg_weight)
    : target(arg_target), weight(arg_weight) { }
    
    bool operator < (const Neighbor &n) const {
        return target < n.target;
    }
};

typedef std::vector<std::set<Neighbor> > adjacency_list_t;

}
#endif /* graph_h */
