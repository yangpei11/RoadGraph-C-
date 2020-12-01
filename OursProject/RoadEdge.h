#pragma once

#include "CommonRoadType.h"
#include "VPMath.h"
#include "RoadFeature.h"
#include <cstdint>
#include <vector>
#include "RoadConstructor.h"

class RoadNode;

/// 表达一条道路的数据结构
class RoadEdge : public RoadFeature
{
	friend RoadGraph;
	friend RoadConstructor;
public:
    RoadEdge(RoadObjectId id, RoadGraph *road_graph);

    ~RoadEdge();

    // 添加一个节点
	void AddNode(RoadNode *node);

	/// \brief 获得另一端的节点
    RoadNode *GetOtherNode(const RoadNode *const node) const;


	// 设置道路属性
    void SetupEdge() {
        SetupLanes();   //设置车道
        SetupWidth();   //设置路宽
        SetupSpeed();   //设置车速
        SetupLights();  //设置路灯
    }
    /// \brief 设置道路总宽度
    void SetupWidth();
    // 设置道路车道数
    void SetupLanes();
    // 设置道路限速
    void SetupSpeed();
    // 设置路灯参数
    void SetupLights();

    std::vector<RoadNode *> nodes_;         //道路节点
    std::vector<VPE::dvec3> global_points_; //全局坐标点

	//道路属性部分：
    float    width_      = -1.0f;  //路宽
    int      lane_nums_  = 0;      //车道线数量
    float    lane_width_ = -1.0f;  //车道宽
    int      max_speed_  = 0.0f;   //限速
    int      layer       = 0;      //立交桥层级
    bool     is_oneway_  = false;  //是否是单行道
    bool     is_bridge_  = false;  //是否是桥
    bool     is_tunnel_  = false;  //是否是隧道
    bool     is_railway_ = false;  //是否是铁路
    RoadType type_       = UNKOWN; //道路类别
    bool     has_light_;           //是否有路灯
    float    light_space_;         //路灯间隔
    float    column_space_;        //桥柱间隔
};