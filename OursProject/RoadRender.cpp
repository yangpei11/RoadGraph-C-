#include "RoadRender.h"
#include "RenderTree.h"
#include "RenderResource.h"
#include "RoadGraphInterface.h"
#include "World.h"
#include "Light.h"
#include "RoadNode.h"
#include "RoadUtils.h"
#include "RoadTextureCollection.h"

StaticRenderer::RenderResource *RoadRender::column_res_ = nullptr;

RoadRender::RoadRender(RoadGraph *roadgraph):roadgraph_(roadgraph) {    
    if (!column_res_)
        CreateColumnResource();
}

void RoadRender::Init() {
    
    ConstructTerrainCells();
    OrganizeMeshesByTerrainCells();
}
void RoadRender::SetModelMatrix(const VPE::dmat4 &mat) {
    gpu_culling_.SetModelMatrix(mat);
}
void RoadRender::CreateColumnResource() {
    //桥柱
    int                       m_circleSegNum = 10;
    float                     radius         = 0.8;
    float                     hi             = -0.2;
    float                     angleStep      = 2 * VPE::PI / m_circleSegNum;
    int                       segNum1        = m_circleSegNum + 1;
    std::vector<VPE::fvec3>   vertices(2 * (m_circleSegNum + 1), VPE::fvec3());
    std::vector<VPE::fvec3>   normals(2 * (m_circleSegNum + 1), VPE::fvec3());
    std::vector<unsigned int> indices(m_circleSegNum * 2 * 3, 0);
    VPE::fvec3                color(0.7, 0.7, 0.6);

    for (int i = 0; i < 2; ++i) {
        for (int a = 0; a < m_circleSegNum; ++a) {
            float angle = a * angleStep;
            float x     = cosf(angle) * radius;
            float y     = sinf(angle) * radius /*/ cosine*/; // TODO: 根据夹角调整y

            size_t idx    = i * segNum1 + a;
            vertices[idx] = VPE::fvec3(x, hi, y);
            normals[idx]  = VPE::normalize(VPE::fvec3(x, 0, y));
        }
        vertices[i * segNum1 + m_circleSegNum] = vertices[i * segNum1]; //重复起始点用于赋纹理坐标
        normals[i * segNum1 + m_circleSegNum] = normals[i * segNum1]; //重复起始点用于赋纹理坐标

        radius += 0.4;
        hi -= 50;

        //产生索引
        if (i > 0) {
            int i1     = i - 1;
            int curIdx = i1 * m_circleSegNum * 2 * 3;
            for (int a = 0; a < m_circleSegNum; ++a) {
                //三角面1
                indices[curIdx]     = i1 * segNum1 + a;
                indices[curIdx + 1] = i1 * segNum1 + a + 1;
                indices[curIdx + 2] = i * segNum1 + a;

                //三角面2
                indices[curIdx + 3] = i1 * segNum1 + a + 1;
                indices[curIdx + 4] = i * segNum1 + a + 1;
                indices[curIdx + 5] = i * segNum1 + a;

                curIdx += 6;
            }
        }
    }

    column_res_ =
        new StaticRenderer::RenderResource("bridge_column", vertices, indices, normals, color);
    StaticRenderer::RenderTree::GetInstance()->CreateLodInstance({column_res_}, {4000});
}

void RoadRender::AddLight(const VPE::dmat4 &mat) {
    /// street light
    if (g_roadgraph_interface.GetLODParameter().show_street_light) {
        std::vector<std::string> models;
        std::vector<double>      dis;
        auto                     street_light = g_roadgraph_interface.GetStreetLightModelParam();
        for (auto &light : street_light) {
            models.push_back(light.name);
            dis.push_back(light.distance);
        }
        StaticRenderer::RenderTree::GetInstance()->CreateLodInstance(models, dis);
        StaticRenderer::RenderTree::GetInstance()->AddLodInstance(models[0], VPE::dvec3(mat[3]),
                                                                  mat);
    }

    /// light flash
    if (g_roadgraph_interface.GetLODParameter().show_light_flash) {
        VPE::World *world = ViWoROOT::World();
        {
            /// point light parameters
            VPE::vec3  color = g_roadgraph_interface.GetStreetLightColor();
            VPE::fvec3 light_color(color.x, color.y, color.z);
            double     glow_size = 0.3;
            double     min_range = 0.1;
            double     max_range = 50;
            VPE::dvec3 position(-40.0, 265, 0.0);

            VPE::dvec3 light_position = VPE::trans_pos(mat, position);
            auto       entity         = world->CreateEntity();
            world->SetPosition(entity, light_position);
            auto &light = world->assign<VPE::Light>(entity, VPE::POINT_LIGHT, light_color);
            light.SetMaxRange(max_range);
            light.SetIntensity(50);
            light.SetGlowSize(glow_size);
            light.SetEnable(false);
            // light.EnableCasting();
            lights_.emplace_back(entity);
        }
    }
}

void RoadRender::AddColumn(const VPE::dmat4 &mat) {
    StaticRenderer::RenderTree::GetInstance()->AddLodInstance(column_res_, mat);
}

void RoadRender::EnableLights(bool enable /* = true */) {
    auto world = ViWoROOT::World();
    for (auto light : lights_) {
        world->get<VPE::Light>(light).SetEnable(enable);
    }
    light_on_ = enable;
}

void RoadRender::Render(void *pOptions, double _time, float _deltatime,
                        const VPE::dvec3 &camera_pos,
                       const std::unordered_set<std::uint64_t> &visible_blocks) {

    gpu_culling_.Culling();
    auto RenderByBlocks =
        [&visible_blocks, this](
            const std::unordered_map<std::uint64_t, std::vector<RoadObjectId>> &block_feature_map) {
            for (auto block : visible_blocks) {
                auto itr = block_feature_map.find(block);
                if (itr == block_feature_map.end())
                    continue;
                auto &feature_ids = itr->second;
                for (auto id : feature_ids) {
                    if (visible_features_.find(id) == visible_features_.end()) {
                        auto edge = roadgraph_->GetRoadEdge(id);
                        visible_features_.insert(id);
                    }
                }
            }
        };
    bool         is_night         = RoadUtils::IsNight();
    auto         LightOn          = [this](bool is_night) -> float { return is_night ? 1.0 : 0.0; };
    auto *       camera           = ViWoROOT::GetCameraManager()->GetCurrCamera();
    VPE::dmat4x4 viewMatrix       = camera->getViewMatrix();
    VPE::dmat4x4 projectionMatrix = camera->getProjectMatrixd();
    if (g_roadgraph_interface.GetLODParameter().show_road_mesh) {
        visible_features_.clear(); //清空已经绘制的集合

        gpu_culling_.Render(viewMatrix, projectionMatrix, LightOn(is_night));
        RenderByBlocks(block_edge_maps_);
        RenderByBlocks(block_node_maps_); //目前node没有生成网格，暂时省略
                                          /*if (!total_columns_.empty())
                                          {
                                              StaticRenderer::RenderTree::GetInstance()->AddInstance(column_res_, total_columns_,
                                          false);
                                          }*/
    }

    if (!roadgraph_->IsLightManualControl() && is_night != light_on_) {
        EnableLights(is_night);
        light_on_ = is_night;
    }

   
}

void RoadRender::ConstructTerrainCells() {
    const RoadLoadParameter &lodparameter = g_roadgraph_interface.GetLODParameter();
    for (auto &kv : roadgraph_->road_edges_) {
        auto &nodes = kv.second->nodes_;
        auto  edge_id = kv.second->GetID();
        for (auto node : nodes) {
            //根据node经纬度生成地形块id
            RoadObjectId key =
                RoadUtils::GenerateKeyFromLonLat(node->lon, node->lat, lodparameter.split_level);
            auto iter = block_edge_maps_.find(key);
            if (iter != block_edge_maps_.end()) {
                
                if (std::find(iter->second.begin(), iter->second.end(), edge_id) ==
                    iter->second.end())
                    iter->second.push_back(edge_id);
            } else {
                block_edge_maps_[key] = std::vector<RoadObjectId>(1, edge_id);
            }
        }
    }
}

void RoadRender::OrganizeMeshesByTerrainCells() {
    //检测只占一个地形块的网格，以按照地形块合并
    std::unordered_map<RoadObjectId, int> feature_count_map;
    for (auto &item : block_edge_maps_) {
        for (auto &fid : item.second) {
            if (feature_count_map.find(fid) != feature_count_map.end()) {
                feature_count_map[fid]++;
            } else {
                feature_count_map[fid] = 1;
            }
        }
    }

    std::vector<GLuint64> textureHandles;
    g_road_texture_collection.GetTextureHandles(textureHandles);
    gpu_culling_.SetTextures(textureHandles);

    auto GenGPUCullingNode = [this](RoadFeature *feature, std::vector<New_Bindless_VertexData> &nodes,
                                    std::vector<unsigned int> &indices, unsigned int &start_index,
                                    std::array<VPE::dvec3, 2> &bbox) {
        auto meshes = feature->GetSurfaceMesh();
        for (auto mesh : meshes) {
            int texHandleIndex = g_road_texture_collection.GetTextureIndex(mesh->tex_name);

            for (size_t i = 0; i < mesh->vertices.size(); ++i) {
                New_Bindless_VertexData tmp;
                tmp.pos           = mesh->vertices[i].pos;
                tmp.normal        = mesh->vertices[i].normal;
                tmp.uv            = mesh->vertices[i].tex_coords;
                tmp.texIndex      = texHandleIndex;
                tmp.nightTexIndex = texHandleIndex + 1;

                nodes.emplace_back(tmp);
            }

            for (size_t i = 0; i < mesh->indexes.size(); ++i) {
                indices.push_back(start_index + mesh->indexes[i]);
            }
            start_index += mesh->vertices.size();
        }
        std::array<VPE::dvec3, 2> aabb = feature->GetBoundingBox();
        for (auto &pt : aabb) {
            pt = roadgraph_->ToGlobal(pt);
        }
        bbox[0] = VPE::min(aabb[0], bbox[0]);
        bbox[1] = VPE::max(aabb[1], bbox[1]);

        feature->ClearMeshData();
    };

    auto GenBoundingBox = [](const std::array<VPE::dvec3, 2> &aabb) -> std::vector<VPE::vec4> {
        std::vector<VPE::vec4> bounding_box;
        bounding_box.resize(8);

        bounding_box[0] = VPE::vec4(aabb[0].x, aabb[0].y, aabb[0].z, 0);
        bounding_box[1] = VPE::vec4(aabb[1].x, aabb[0].y, aabb[0].z, 0);
        bounding_box[2] = VPE::vec4(aabb[1].x, aabb[1].y, aabb[0].z, 0);
        bounding_box[3] = VPE::vec4(aabb[0].x, aabb[1].y, aabb[0].z, 0);

        bounding_box[4] = VPE::vec4(aabb[0].x, aabb[0].y, aabb[1].z, 0);
        bounding_box[5] = VPE::vec4(aabb[1].x, aabb[0].y, aabb[1].z, 0);
        bounding_box[6] = VPE::vec4(aabb[1].x, aabb[1].y, aabb[1].z, 0);
        bounding_box[7] = VPE::vec4(aabb[0].x, aabb[1].y, aabb[1].z, 0);

        return bounding_box;
    };

    for (auto &item : block_edge_maps_) //遍历地形块
    {
        std::vector<New_Bindless_VertexData> nodes;
        std::vector<unsigned int>            indices;
        unsigned int                    start_index = 0;
        std::array<VPE::dvec3, 2>       aabb;
        aabb[0] = VPE::dvec3(DBL_MAX, DBL_MAX, DBL_MAX);
        aabb[1] = VPE::dvec3(-DBL_MAX, -DBL_MAX, -DBL_MAX);

        for (auto &fid : item.second) //遍历地形块内的edge
        {
            RoadFeature *feature = roadgraph_->GetRoadEdge(fid);

            if (feature_count_map[fid] == 1) {
                GenGPUCullingNode(feature, nodes, indices, start_index, aabb);
            } else {
                std::vector<New_Bindless_VertexData> tmp_nodes;
                std::vector<unsigned int>            tmp_indices;
                unsigned int                    tmp_start_index = 0;
                std::array<VPE::dvec3, 2>       tmp_aabb;
                tmp_aabb[0] = VPE::dvec3(DBL_MAX, DBL_MAX, DBL_MAX);
                tmp_aabb[1] = VPE::dvec3(-DBL_MAX, -DBL_MAX, -DBL_MAX);

                GenGPUCullingNode(feature, tmp_nodes, tmp_indices, tmp_start_index, tmp_aabb);

                gpu_culling_.AddNode(tmp_nodes, tmp_indices, GenBoundingBox(tmp_aabb));
            }
        }

        gpu_culling_.AddNode(nodes, indices, GenBoundingBox(aabb));
    }
}