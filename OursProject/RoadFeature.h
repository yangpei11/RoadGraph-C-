#pragma once
#include <unordered_map>
#include <array>
#include "CommonRoadType.h"
#include "boost/variant.hpp"
#include "boost/serialization/variant.hpp"
#include "boost/serialization/unordered_map.hpp"


class RoadGraph;
struct Lane;

typedef boost::variant<int, float, bool, std::string> RoadVar;

//道路元素（RoadEdge与Intersection）的基类
class RoadFeature
{
public:
	RoadFeature(RoadObjectId id, RoadGraph * road_graph = nullptr);
	virtual ~RoadFeature();

	RoadObjectId GetID() const { return id_; }

	const std::vector<RoadMesh*>& GetSurfaceMesh() const {
		return road_mesh_;
	}

	void ClearMeshData();

	void SetRoadGraph(RoadGraph* graph);
	const RoadGraph* GetRoadGraph() const { return road_graph_; }

	const std::array<VPE::dvec3, 2> & GetBoundingBox() const {
		return aabb_;
	}

	void UpdateBBox(const VPE::dvec3 & vert);

protected:
	RoadObjectId id_;
	std::vector<Lane*> lanes_;			//车道线
	std::vector<RoadMesh*> road_mesh_;	//一个feature可以有多个mesh
	RoadGraph * road_graph_;			//所从属的roadgraph
	std::array<VPE::dvec3, 2> aabb_; //bounding box, in local coordinates
};
