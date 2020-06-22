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
	// ������·
	void Construct(RoadGraph& roadgraph);
private:
	
	void PreparePoints(RoadGraph& roadgraph);		//׼������
	void BuildTopology(RoadGraph& roadgraph);		//��������
	//void CleanTempData(RoadGraph* roadgraph);		//������ʱ����

	void ConstructIntersection(RoadGraph& roadgraph);	//��������·��
    const std::vector<VPE::dvec2> & ConstructArcRoads(RoadEdge & edge);		//ArcRoadƽ����·������
    //������ά�ṹ
	void Construct3D(RoadEdge &edge, const std::vector<VPE::dvec2> &points2d); 
	//�����������
    void InsertPointsToGround(RoadEdge &edge);

	//��������  
	void ConstructMesh(RoadEdge &edge);                                        
    //������·��Ե
    void CalcCurbs(RoadMesh &mesh, const VPE::dvec3 &pt, const VPE::dvec3 &up,
                   const VPE::dvec3 &split_dir, float len);
    //����·��λ����̬��Ϣ
    void CalcRoadLightData(RoadEdge &edge, float road_left_width, float road_right_width);
    //��������
    void CalcBridgeColumn(RoadEdge &edge, float road_left_width, float road_right_width);

	// void ConstructLanes(RoadGraph* roadgraph);		//����������

	////��line����ƫ��offset(������)���������offset_line
	void OffsetLine(const std::vector<VPE::dvec3>& line, std::vector<VPE::dvec3> & offset_line, float offset);

	////debug draw functions
    void ShowRoadNodes(RoadGraph &roadgraph);
    void ShowIntersections(RoadGraph &roadgraph);
    // void ShowLanes(RoadGraph* roadgraph);

	float offset_to_terrain_;

	VPE::RenderModule::IVectorGraphicsRender* vg_;

	VPE::IDAllocator<RoadObjectId> id_alloc; //id������
    std::vector<RoadNode *>        intersection_nodes;

    std::vector<RoadIntersection *> Intersections;
    std::vector<ArcRoad *>         arc_roads;
};

