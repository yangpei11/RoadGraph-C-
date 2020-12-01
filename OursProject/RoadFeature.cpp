#include "RoadFeature.h"
#include <vector>
#include "RoadUtils.h"

using namespace std;

RoadFeature::RoadFeature(RoadObjectId id, RoadGraph * road_graph)
	:road_graph_(road_graph), id_(id)
{
}


RoadFeature::~RoadFeature()
{
	ClearMeshData();
}

void RoadFeature::ClearMeshData()
{
	for (auto mesh : road_mesh_)
	{
		delete mesh;
	}
	road_mesh_.clear();
}

void RoadFeature::SetRoadGraph(RoadGraph * graph)
{
	assert(graph != nullptr);
	road_graph_ = graph;
}

void RoadFeature::UpdateBBox(const VPE::dvec3 & vert)
{
	aabb_[0] = VPE::min(aabb_[0], vert);
	aabb_[1] = VPE::max(aabb_[1], vert);
}