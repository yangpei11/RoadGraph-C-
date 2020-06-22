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

//�����·������ص���
class RoadRender {

public:
    RoadRender(RoadGraph * roadgraph);

    void Init();
    //���õ�·ģ�;���
    void SetModelMatrix(const VPE::dmat4 &mat);
    //���·��
    void AddLight(const VPE::dmat4 &mat);
    //�������
    void AddColumn(const VPE::dmat4 &mat);
    //����·��
    void EnableLights(bool enable = true);

    void Render(void *pOptions, double _time, float _deltatime, const VPE::dvec3 &camera_pos,
                const std::unordered_set<std::uint64_t> &block_index);

    const std::unordered_set<RoadObjectId> &GetVisibleFeatures() const {
        return visible_features_;
    }

private:

    void CreateColumnResource(); //����������Դ

    void ConstructTerrainCells(); //���ݾ�γ�Ƚ���·Ԫ�ع鵽�����ο���֯�Ľṹ��
    void OrganizeMeshesByTerrainCells(); //���ݵ��ο���֯Mesh����

    RoadGraph *                            roadgraph_;
    std::vector<VPE::Entity>               lights_;
    static StaticRenderer::RenderResource *column_res_; //������Դ
    NewGPUCullingModel                     gpu_culling_;
    std::unordered_map<std::uint64_t, std::vector<RoadObjectId>>
        block_edge_maps_; //��������id��ֵ��RoadEdge id
    std::unordered_map<std::uint64_t, std::vector<RoadObjectId>>
        block_node_maps_; //��������id��ֵ��IntersectionPoint id
    std::unordered_set<RoadObjectId> visible_features_; //�Ѿ����Ƶļ���
    bool                              light_on_;         //��ǰ�ƹ�״̬
};
