//
//  dynamic_airway_graph.hpp
//  DWR
//
//  Created by ZachQin on 2017/3/7.
//  Copyright © 2017年 Zach. All rights reserved.
//

#ifndef dynamic_airway_graph_hpp
#define dynamic_airway_graph_hpp

#include "airway_graph.hpp"
#include <stdio.h>
#include <set>

namespace dwr {

struct UndirectedAirwayPointPair {
    std::shared_ptr<AirwayPoint> first;
    std::shared_ptr<AirwayPoint> second;
    
    UndirectedAirwayPointPair(const AirwayPointPair &pair) : first(std::min(pair.first, pair.second)), second(std::max(pair.first, pair.second)) {};
    UndirectedAirwayPointPair(const std::shared_ptr<AirwayPoint> &ap1, const std::shared_ptr<AirwayPoint> &ap2) : first(std::min(ap1, ap2)), second(std::max(ap1, ap2)) {};
        
    bool operator < (const UndirectedAirwayPointPair &p) const {
        return this->first < p.first ? true : (this->first > p.first ? false : this->second < p.second);
    };
};
    
class DynamicAirwayGraph: public AirwayGraph {
public:
    std::vector<std::shared_ptr<AirwayPoint>> GetDynamicPath(AirwayPointID sourceIdentity, AirwayPointID destinIdentity);
    void ForEachBlock(std::function<void(const AirwayPoint &, const AirwayPoint &)> &traverseFunction);
    
protected:
    std::set<UndirectedAirwayPointPair> blockSet_;
};

}
#endif /* dynamic_airway_graph_hpp */
