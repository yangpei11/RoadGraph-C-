#pragma once
#include <unordered_map>
#include <vector>
#include "CommonRoadType.h"
#include "glm/glm.hpp"
#include "IDAllocator.h"

class RoadGraph;
class RoadEdge;
class RoadNode;
class RoadIntersection;
class ArcRoad;
namespace VPE
{
	namespace RenderModule {
		class IVectorGraphicsRender;
	}
}


class RoadConstructor
{
public:
	RoadConstructor();
	// 构建道路
	void Construct(RoadGraph& roadgraph);
private:
	
	void PreparePoints(RoadGraph& roadgraph);		//准备数据
	void BuildTopology(RoadGraph& roadgraph);		//建立拓扑
	//void CleanTempData(RoadGraph* roadgraph);		//清理临时数据

	void ConstructIntersection(RoadGraph& roadgraph);	//构建交叉路口
    const std::vector<VPE::dvec2> & ConstructArcRoads(RoadEdge & edge);		//ArcRoad平滑道路中轴线
    //构建三维结构
	void Construct3D(RoadEdge &edge, const std::vector<VPE::dvec2> &points2d); 
	//插入点以贴地
    void InsertPointsToGround(RoadEdge &edge);

	//构建网格  
	void ConstructMesh(RoadEdge &edge);                                        
    //产生道路边缘
    void CalcCurbs(RoadMesh &mesh, const VPE::dvec3 &pt, const VPE::dvec3 &up,
                   const VPE::dvec3 &split_dir, float len);
    //计算路灯位置姿态信息
    void CalcRoadLightData(RoadEdge &edge, float road_left_width, float road_right_width);
    //产生桥柱
    void CalcBridgeColumn(RoadEdge &edge, float road_left_width, float road_right_width);

	// void ConstructLanes(RoadGraph* roadgraph);		//构建车道线

	////将line进行偏移offset(左负右正)，结果存至offset_line
	void OffsetLine(const std::vector<VPE::dvec3>& line, std::vector<VPE::dvec3> & offset_line, float offset);

	////debug draw functions
    void ShowRoadNodes(RoadGraph &roadgraph);
    void ShowIntersections(RoadGraph &roadgraph);
    // void ShowLanes(RoadGraph* roadgraph);

	float offset_to_terrain_;

	VPE::RenderModule::IVectorGraphicsRender* vg_;

	VPE::IDAllocator<RoadObjectId> id_alloc; //id分配器
    std::vector<RoadNode *>        intersection_nodes;

    std::vector<RoadIntersection *> Intersections;
    std::vector<ArcRoad *>         arc_roads;
};

