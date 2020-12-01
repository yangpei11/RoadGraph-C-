#pragma once

#include "CommonRoadType.h"
#include "VPMath.h"
#include "RoadFeature.h"
#include <cstdint>
#include <vector>
#include "RoadConstructor.h"

class RoadNode;

/// ���һ����·�����ݽṹ
class RoadEdge : public RoadFeature
{
	friend RoadGraph;
	friend RoadConstructor;
public:
    RoadEdge(RoadObjectId id, RoadGraph *road_graph);

    ~RoadEdge();

    // ���һ���ڵ�
	void AddNode(RoadNode *node);

	/// \brief �����һ�˵Ľڵ�
    RoadNode *GetOtherNode(const RoadNode *const node) const;


	// ���õ�·����
    void SetupEdge() {
        SetupLanes();   //���ó���
        SetupWidth();   //����·��
        SetupSpeed();   //���ó���
        SetupLights();  //����·��
    }
    /// \brief ���õ�·�ܿ��
    void SetupWidth();
    // ���õ�·������
    void SetupLanes();
    // ���õ�·����
    void SetupSpeed();
    // ����·�Ʋ���
    void SetupLights();

    std::vector<RoadNode *> nodes_;         //��·�ڵ�
    std::vector<VPE::dvec3> global_points_; //ȫ�������

	//��·���Բ��֣�
    float    width_      = -1.0f;  //·��
    int      lane_nums_  = 0;      //����������
    float    lane_width_ = -1.0f;  //������
    int      max_speed_  = 0.0f;   //����
    int      layer       = 0;      //�����Ų㼶
    bool     is_oneway_  = false;  //�Ƿ��ǵ��е�
    bool     is_bridge_  = false;  //�Ƿ�����
    bool     is_tunnel_  = false;  //�Ƿ������
    bool     is_railway_ = false;  //�Ƿ�����·
    RoadType type_       = UNKOWN; //��·���
    bool     has_light_;           //�Ƿ���·��
    float    light_space_;         //·�Ƽ��
    float    column_space_;        //�������
};