#pragma once

#include "IRoadGraphInterface.h"
#include "CommonRoadType.h"
#include "RoadGraph.h"

#include "VPMath.h"

#include "boost/asio.hpp"
#include "boost/serialization/list.hpp"
#include "boost/serialization/string.hpp"
#include "boost/serialization/vector.hpp"
#include "boost/serialization/unordered_map.hpp"
#include "boost/serialization/utility.hpp"
#include "boost/serialization/split_member.hpp"
#include "boost/archive/binary_iarchive.hpp"
#include "boost/archive/binary_oarchive.hpp"
#include "boost/archive/xml_iarchive.hpp"
#include "boost/archive/xml_oarchive.hpp"
#include "boost/thread/mutex.hpp"

#include "Renderable.h"
#include "SceneManager.h"

#include <unordered_map>
#include "GLProgramObject.h"

#include <vector>
#include <string>
#include "OSMDataLoader.h"
#include "FMainLoop.h"
#include "RoadGraphLib.h"


class CNetWorkCacheManager;

class _ROADGRAPH_EXP RoadGraphInterface : public IRoadGraphInterface, public VPE::RenderSystem::IRenderable, public VPE::FMainLoop, public VPE::SceneModule
{
public:
    RoadGraphInterface();

    virtual ~RoadGraphInterface();

	bool Load(const std::string & data_file, std::function<void(int percent)> callback = {}) override;
	bool Release(const std::string & data_file) override;
	bool Clear() override;

    void Render( void *pOptions /* = 0 */, double _time /* = 0.0 */, float _deltatime /* = 0.0f */ );

    virtual RoadObjectId CreateRoadGraph();

    virtual bool DeleteGraph(RoadObjectId graph_id);

    virtual bool LoadConfig(RoadObjectId graph_id, const std::string& filename);

    virtual RoadObjectId ImportOSM(const std::string& filename);

    virtual void HideGraph( RoadObjectId graph_id );

    virtual void ShowGraph( RoadObjectId graph_id );

    virtual void ClearAllRoadGraph();

    virtual void ConstructGraphNode( RoadObjectId node_id);

    virtual void ConstructAll();

    virtual void StartVehicleSchedule();

    double GetOneWayBaseSpeed() const { return vehicle_oneway_base_speed_; }

    double GetNormalWayBaseSpeed() const { return vehicle_normalway_bse_speed_; }

    const RoadLoadParameter& GetLODParameter() const { return load_param_; }

    //RoadEdge* GetEdgeCacheFromID(RoadObjectId graph_id, RoadObjectId edge_id);

    unsigned int GetControlCameraVehicleNum() const { return control_camera_vehicle_num_; }

    const std::vector<ModelType>& GetStreetLightModelParam() const { return street_light_model_; }

    const double GetStartShowVehicleDistance() const { return start_show_vehicle_distance_; }

    void OpenAllStreetLight(bool is_open) override;

    const GLProgramObject& GetRoadGraphProgram() { return road_graph_program_; }

    const GLProgramObject& GetRoadGraphNightProgram() { return road_graph_night_program_; }

    CNetWorkCacheManager* GetDatabaseHandler(RoadObjectId graph_id);

    bool LoadRoadResources(const std::string& filename);

    virtual bool Load(std::vector<std::string> &filenames);

    virtual bool Unload();

    virtual int OnEvent(double _curtime, double _deltatime, void *_userdata) override;

	RoadGraph* GetRoadGraph(RoadObjectId graph_id);

	const VPE::vec3 & GetStreetLightColor() const;

	float GetStreetLightSpace() const;

	void SetVisible(bool visible);

protected:
    

    //void SaveDB(const std::string & db_filename);

    std::vector<RoadObjectId> LoadDB(const std::string& db_filename);

    void ComputeVisibleTerrainBlocks(std::unordered_set<std::uint64_t>& block_index);

    std::unordered_map<RoadObjectId, RoadGraph*> road_graphs_;

    double vehicle_oneway_base_speed_;
    double vehicle_normalway_bse_speed_;

    RoadLoadParameter load_param_;

	std::vector<ModelType> street_light_model_;

	VPE::vec3 street_light_color_;

	float street_light_space_;

    double start_show_vehicle_distance_;

    unsigned int control_camera_vehicle_num_;

    // 用于暂存现有地形绘制块信息
    std::vector<VPE::ivec3> renderInfoVec;

    std::unordered_map<RoadObjectId, CNetWorkCacheManager*> db_handle_map_;

    bool data_from_cache_;

    GLProgramObject road_graph_program_;

    GLProgramObject road_graph_night_program_;

    boost::mutex road_graphs_mutex_;

    boost::mutex db_handle_mutex_;
};

extern RoadGraphInterface g_roadgraph_interface;