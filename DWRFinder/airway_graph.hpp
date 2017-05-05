//
//  AirwayGraph.hpp
//  DWR
//
//  Created by ZachQin on 2017/3/3.
//  Copyright © 2017年 Zach. All rights reserved.
//

#ifndef airway_graph_hpp
#define airway_graph_hpp

#include <stdio.h>
#include <map>
#include <vector>
#include "airway_type.hpp"

namespace dwr {

typedef std::pair<const std::shared_ptr<AirwayPoint>, const std::shared_ptr<AirwayPoint>> AirwayPointPair;
    
class AirwayGraph {
public:
    AirwayGraph(){};
    AirwayGraph(const char *path);
    void AddAirwayPoint(AirwayPointID identity, std::string name, double lon, double lat);
    void AddAirwaySegment(AirwayPointID identity1, AirwayPointID identity2);
    std::vector<std::shared_ptr<AirwayPoint>> GetPath(AirwayPointID sourceIdentity, AirwayPointID destinIdentity, const std::function<bool(const AirwayPointPair &)> canSearch = [](const AirwayPointPair &p){return true;});
    bool SaveToFile(const std::string &path) const;
    bool LoadFromFile(const std::string &path);
    void ForEach(std::function<void(const std::shared_ptr<AirwayPoint> &, const std::shared_ptr<AirwayPoint> &, GeoDistance)> traverseFunction);
    std::shared_ptr<AirwayPoint> AirwayPointFromID(AirwayPointID identity);

protected:
    std::map<AirwayPointID, std::shared_ptr<AirwayPoint>> airwayPointMap_;
};

}
#endif /* AirwayGraph_hpp */
