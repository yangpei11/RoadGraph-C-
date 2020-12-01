#include "RoadGraph.h"
#include "RoadNode.h"
#include "CNetWorkCacheManager.h"
#include "RoadGraphInterface.h"
#include "RoadUtils.h"

#include "ITerrain.h"
#include "RenderEngine/RenderEngine.h"
#include "RenderSystemInterface.h"
#include "RenderTree.h"
#include "TerrainExport/TerrainCacheIndex.h"

#include "CommonRoadType.h"
#include "GL/glew.h"
#include "Light.h"
#include "RenderResource.h"
#include "RoadTextureCollection.h"
#include "VehicleSchedule.h"
#include "World.h"
#include "boost/asio.hpp"
#include "boost/foreach.hpp"
#include <algorithm>
#include <map>
#include "RoadRender.h"

using namespace std;
using namespace glm;

#ifdef min
#undef min
#endif // min
#ifdef max
#undef max
#endif // max

RoadGraph::RoadGraph(RoadObjectId graph_id)
    : graph_id_(graph_id)
    , lights_manual_ctrl_(false) {
    render_ = std::make_unique<RoadRender>(this);
}

RoadGraph::~RoadGraph() {
    for_each(road_edges_.begin(), road_edges_.end(),
             [](pair<RoadObjectId, RoadEdge *> e) { delete e.second; });
}

void RoadGraph::AddRoadNode(RoadNode *node) {
    assert(node != nullptr);
    road_nodes_[node->id] = node;
}

void RoadGraph::AddRoadEdge(RoadEdge *edge) {
    assert(edge != nullptr);
    edge->SetRoadGraph(this);
    road_edges_[edge->id_] = edge;
    
}

RoadNode *RoadGraph::GetRoadNode(RoadObjectId id) {
    auto it = road_nodes_.find(id);
    if (it != road_nodes_.end()) {
        return it->second;
    } else {
        return nullptr;
    }
}

RoadEdge *RoadGraph::GetRoadEdge(RoadObjectId id) {
    auto it = road_edges_.find(id);
    if (it != road_edges_.end()) {
        return it->second;
    } else {
        return nullptr;
    }
}


bool RoadGraph::IsVisible(const RoadFeature *const ft) const {
    assert(ft != nullptr);
    return (render_->GetVisibleFeatures().find(ft->GetID()) != render_->GetVisibleFeatures().end());
}

const unordered_set<RoadObjectId> &RoadGraph::GetVisibleFeatures() const {
    return render_->GetVisibleFeatures();
}

void RoadGraph::SaveDB(CNetWorkCacheManager &db_handle) {
    ////存储所有边信息
    //BOOST_FOREACH (auto it, road_edges_) {
    //    string            key       = boost::lexical_cast<string>(it.first);
    //    std::vector<char> edge_data = it.second->Serialization();

    //    db_handle.AddCacheData(key, edge_data.data(), edge_data.size(), true);
    //}

    ////存储road_graph其他辅助信息
    //std::string                                       serial_str;
    //boost::iostreams::back_insert_device<std::string> inserter(serial_str);
    //boost::iostreams::stream<boost::iostreams::back_insert_device<std::string>> s(inserter);
    //boost::archive::binary_oarchive                                             oa(s);

    //oa << *this;
    //s.flush();

    //std::string graph_id_str = boost::lexical_cast<string>(graph_id_);
    //db_handle.AddCacheData(graph_id_str, const_cast<char *>(serial_str.c_str()), serial_str.size(),
    //                       true);
}

void RoadGraph::LoadDB(CNetWorkCacheManager &db_handle) {

    //for (auto block : block_edge_maps_) {
    //    auto &features = block.second;
    //    for (auto fid : features) {
    //        auto feature = GetRoadEdge(fid);
    //        if (feature == nullptr) {
    //            auto way = g_roadgraph_interface.GetEdgeCacheFromID(graph_id_, fid);
    //            if (way) {
    //                road_edges_.insert(make_pair(fid, way));
    //                way->SetRoadGraph(this);
    //            }
    //        }
    //    }
    //}
}

void RoadGraph::SetCenter(double lon, double lat) {
    dvec3 p_geo(lon, lat, 0.0);
    dvec3 global_point;
    g_coord.LongLat2GlobalCoord(value_ptr<double>(p_geo), value_ptr<double>(global_point));

    //构建局部坐标系，x为北，y为地表法向，z为东
    dvec3 local_y = normalize(global_point);
    dvec3 local_z = normalize(cross(dvec3(0.0, 0.0, 1.0), local_y));
    dvec3 local_x = normalize(cross(local_y, local_z));

    model_matrix_ =
        dmat4(local_x.x, local_x.y, local_x.z, 0, local_y.x, local_y.y, local_y.z, 0, local_z.x,
              local_z.y, local_z.z, 0, global_point.x, global_point.y, global_point.z, 1);    
    w2l_matrix_ = inverse(model_matrix_);
    render_->SetModelMatrix(model_matrix_);
}

void RoadGraph::Init() {
    render_->Init();
}

VPE::dvec3 RoadGraph::ToLocal(const VPE::dvec3 &global, bool is_point /* = true */) const {
    VPE::dvec4 vec4(global, is_point ? 1.0 : 0.0);
    return VPE::dvec3((w2l_matrix_ * vec4).xyz);
}

VPE::dvec3 RoadGraph::ToGlobal(const VPE::dvec3 &local, bool is_point /* = true */) const {
    VPE::dvec4 vec4(local, is_point ? 1.0 : 0.0);
    return VPE::dvec3((model_matrix_ * vec4).xyz);
}

void RoadGraph::OpenStreetLight(bool is_open /* = true */) {
    lights_manual_ctrl_ = true;
    render_->EnableLights(is_open);
    
}

void RoadGraph::AddLight(const VPE::dmat4 &mat) {
    render_->AddLight(mat);
}

void RoadGraph::AddColumn(const VPE::dmat4 &mat) {
    render_->AddColumn(mat);
}