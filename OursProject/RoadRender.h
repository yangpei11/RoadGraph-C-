#pragma once
#include "VPMath.h"
#include "Entity.h"
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include "RenderEngine/NewGPUCullingModel.h"
#include "CommonRoadType.h"

namespace StaticRenderer {
class RenderResource;
}
class RoadGraph;

//负责道路绘制相关的类
class RoadRender {

public:
    RoadRender(RoadGraph * roadgraph);

    void Init();
    //设置道路模型矩阵
    void SetModelMatrix(const VPE::dmat4 &mat);
    //添加路灯
    void AddLight(const VPE::dmat4 &mat);
    //添加桥柱
    void AddColumn(const VPE::dmat4 &mat);
    //开启路灯
    void EnableLights(bool enable = true);

    void Render(void *pOptions, double _time, float _deltatime, const VPE::dvec3 &camera_pos,
                const std::unordered_set<std::uint64_t> &block_index);

    const std::unordered_set<RoadObjectId> &GetVisibleFeatures() const {
        return visible_features_;
    }

private:

    void CreateColumnResource(); //产生桥柱资源

    void ConstructTerrainCells(); //根据经纬度将道路元素归到按地形块组织的结构中
    void OrganizeMeshesByTerrainCells(); //根据地形块组织Mesh数据

    RoadGraph *                            roadgraph_;
    std::vector<VPE::Entity>               lights_;
    static StaticRenderer::RenderResource *column_res_; //桥柱资源
    NewGPUCullingModel                     gpu_culling_;
    std::unordered_map<std::uint64_t, std::vector<RoadObjectId>>
        block_edge_maps_; //键：地形id，值：RoadEdge id
    std::unordered_map<std::uint64_t, std::vector<RoadObjectId>>
        block_node_maps_; //键：地形id，值：IntersectionPoint id
    std::unordered_set<RoadObjectId> visible_features_; //已经绘制的集合
    bool                              light_on_;         //当前灯光状态
};
