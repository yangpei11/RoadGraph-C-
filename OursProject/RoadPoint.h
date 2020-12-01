#pragma once
#include "CommonRoadType.h"
#include <vector>
class RoadEdge;
class RoadNode;
class RoadPoint
{
public:
	RoadPoint(RoadObjectId i, double lo, double la, double al)
		:id(i), lon(lo), lat(la), alt(al){}
	RoadPoint()
		:id(0), lon(0.0), lat(0.0), alt(0.0){}

	union
	{
		glm::dvec3 geo_coord_;
		struct { double lon, lat, alt; };
	};

	RoadObjectId id;
};