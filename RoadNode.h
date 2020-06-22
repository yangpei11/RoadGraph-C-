#pragma once
#include "CommonRoadType.h"
#include <vector>

class RoadEdge;
class RoadNode
{
public:
    RoadNode()
        : id(0) {
    }
	RoadNode(RoadObjectId ID, double lon, double lat, double alt) {
		id = ID;
		lonLatAlt = VPE::dvec3(lon, lat, alt);
	}

	RoadNode(RoadObjectId ID, const VPE::dvec3 &lonLat) {
		id = ID;
		lonLatAlt = lonLat;
	}
    union {
        VPE::dvec3 lonLatAlt;
        struct {
            double lon, lat, alt;
        };
	};
	
	VPE::dvec3 localCoord;
	std::vector<RoadNode*> connectNodes; // 连接的节点
	std::vector<RoadEdge*> edges; // 属于的道路
	RoadObjectId id;
};

