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
    //��ȡ�����ݺ���г�ʼ��
    void Init();
    //��ӽڵ�
    void AddRoadNode(RoadNode* node);
    //��ӵ�·
    void AddRoadEdge(RoadEdge* edge);
    //����id��ýڵ㣬�������ڷ���nullptr
    RoadNode *GetRoadNode(RoadObjectId id);
    //����id��õ�·���������ڷ���nullptr
    RoadEdge* GetRoadEdge(RoadObjectId id);

	/// \brief ��������ת�ֲ����꣬is_point ����ǵ㻹������
	VPE::dvec3 ToLocal(const VPE::dvec3 & global, bool is_point = true) const;

	/// \brief �ֲ�����ת�������꣬is_point ����ǵ㻹������
	VPE::dvec3 ToGlobal(const VPE::dvec3 & local, bool is_point = true) const;

    RoadObjectId GetID() const { return graph_id_; }

    void SaveDB(CNetWorkCacheManager& db_handle);
	void LoadDB(CNetWorkCacheManager& db_handle);

	bool IsVisible(const RoadFeature * const ft) const;

	const std::unordered_set<RoadObjectId> &GetVisibleFeatures() const;

	/// \brief �������ĵ���ȷ���ֲ�����ϵ
	void SetCenter(double lon, double lat);
	// \brief ����·��
	void OpenStreetLight(bool is_open = true);

	//���·��
	void AddLight(const VPE::dmat4 & mat);
    //�������
	void AddColumn(const VPE::dmat4 &mat);
    //�Ƿ����ֶ�����·�ƿ���
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
	VPE::dmat4 model_matrix_;		//ģ�;��󣬾ֲ�����ת��������
	VPE::dmat4 w2l_matrix_;			//��������ת�ֲ�����ϵ�ľ��󣬼�model_matrix_�������
    bool lights_manual_ctrl_; //�Ƿ����ֶ�����·�ƿ���
	
    std::unique_ptr<RoadRender> render_;
};

BOOST_CLASS_VERSION(RoadGraph, 1);

