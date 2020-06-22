#pragma once
#include "VPMath.h"
#include "RoadEdge.h"
#include "CommonRoadType.h"

#include "boost/asio.hpp"
#include "boost/foreach.hpp"
#include "boost/serialization/list.hpp"
#include "boost/serialization/string.hpp"
#include "boost/serialization/vector.hpp"
#include "boost/serialization/unordered_map.hpp"
#include "boost/serialization/utility.hpp"

#include "boost/iostreams/stream.hpp"
#include "boost/iostreams/device/back_inserter.hpp"
#include "boost/lexical_cast.hpp"
#include "boost/atomic/atomic.hpp"

#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "VPMath.h"
#include "Coordination.h"
#include "ViWoRoot.h"
#include "Camera.h"
#include "CameraManager.h"
#include "EffectManagerInterface.h"



static const int MAX_BLOCK_CACHE_SIZE = 10000;
static const float MAX_FADE_OUT_HEIGHT = 10000.0;
static const float MIN_FADE_OUT_HEIGHT = 5000.0;

class CNetWorkCacheManager;

namespace VPE
{
	class Light;
}
class RoadRender;

class RoadGraph
{
    friend RoadConstructor;
    friend RoadRender;
public:
    //RoadGraph()
    //    : graph_id_(0)
    //    , manual_ctrl_(false) {
    //}

    RoadGraph(RoadObjectId graph_id);

    ~RoadGraph();
    //读取完数据后进行初始化
    void Init();
    //添加节点
    void AddRoadNode(RoadNode* node);
    //添加道路
    void AddRoadEdge(RoadEdge* edge);
    //根据id获得节点，若不存在返回nullptr
    RoadNode *GetRoadNode(RoadObjectId id);
    //根据id获得道路，若不存在返回nullptr
    RoadEdge* GetRoadEdge(RoadObjectId id);

	/// \brief 世界坐标转局部坐标，is_point 标记是点还是向量
	VPE::dvec3 ToLocal(const VPE::dvec3 & global, bool is_point = true) const;

	/// \brief 局部坐标转世界坐标，is_point 标记是点还是向量
	VPE::dvec3 ToGlobal(const VPE::dvec3 & local, bool is_point = true) const;

    RoadObjectId GetID() const { return graph_id_; }

    void SaveDB(CNetWorkCacheManager& db_handle);
	void LoadDB(CNetWorkCacheManager& db_handle);

	bool IsVisible(const RoadFeature * const ft) const;

	const std::unordered_set<RoadObjectId> &GetVisibleFeatures() const;

	/// \brief 设置中心点以确立局部坐标系
	void SetCenter(double lon, double lat);
	// \brief 开关路灯
	void OpenStreetLight(bool is_open = true);

	//添加路灯
	void AddLight(const VPE::dmat4 & mat);
    //添加桥柱
	void AddColumn(const VPE::dmat4 &mat);
    //是否是手动控制路灯开关
	bool IsLightManualControl() const {
        return lights_manual_ctrl_;
    }
    RoadRender *GetRender() {
        return render_.get();
    }

    std::unordered_map<RoadObjectId, RoadEdge *> road_edges_;
    std::unordered_map<RoadObjectId, RoadNode *> road_nodes_;

private:
    RoadObjectId graph_id_;
	VPE::dmat4 model_matrix_;		//模型矩阵，局部坐标转世界坐标
	VPE::dmat4 w2l_matrix_;			//世界坐标转局部坐标系的矩阵，即model_matrix_的逆矩阵
    bool lights_manual_ctrl_; //是否是手动控制路灯开关
	
    std::unique_ptr<RoadRender> render_;
};

BOOST_CLASS_VERSION(RoadGraph, 1);

