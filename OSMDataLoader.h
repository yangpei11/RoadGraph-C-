#pragma once

#include <string>
#include <unordered_map>
#include "CommonRoadType.h"

class RoadGraph;

struct OSMNode
{
	int64_t id;

	double lon;
	double lat;
};

class OSMDataLoader
{
public:
	//return nullptr if failed
    static bool LoadOSM(const std::string& filename, RoadGraph & road_graph);

private:
    static std::unordered_map<std::string, RoadType> type_map;
};