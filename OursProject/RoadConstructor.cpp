#include "RoadConstructor.h"
#include "RoadGraph.h"
#include "RoadNode.h"
#include "Lane.h"
#include "ViWoRoot.h"
#include "ITerrain.h"
#include "RenderEngine/IVectorGraphicsRender.h"
#include "RoadUtils.h"
#include "RoadIntersection.h"
#include "ArcRoad.h"
#include "RoadGraphInterface.h"


RoadConstructor::RoadConstructor()
	:offset_to_terrain_(1.0f)
{
	vg_ = ViWoROOT::GetVectorGraphicsRender();
}

void RoadConstructor::Construct(RoadGraph& roadgraph)
{
	PreparePoints(roadgraph);
	BuildTopology(roadgraph);
	//ShowRoadNodes(roadgraph);	//for debug
	ConstructIntersection(roadgraph);
    //ShowIntersections(roadgraph); //for debug
	for (auto &item : roadgraph.road_edges_) {
        RoadEdge &edge = *item.second;
        auto & arcroad_points = ConstructArcRoads(edge);
        Construct3D(edge, arcroad_points);
        if (g_roadgraph_interface.GetLODParameter().show_road_mesh)
            ConstructMesh(edge);
    }
	
	//ConstructLanes(roadgraph);
	//ShowLanes(roadgraph);		//for debug
}

void RoadConstructor::PreparePoints(RoadGraph& roadgraph)
{
    for (auto &item : roadgraph.road_nodes_) {
        auto  &     node = item.second;
        VPE::dvec3 cur_global;
        g_coord.LongLat2GlobalCoord(node->lonLatAlt, cur_global);
        auto LocalPosition = roadgraph.ToLocal(cur_global);
        node->localCoord  = LocalPosition; //局部坐标xz是水平面， y是高度
    }
}

void RoadConstructor::BuildTopology(RoadGraph& roadgraph)
{
    for (auto &road_point : roadgraph.road_nodes_) {
        auto node = road_point.second;
        /*
        if (node->edges.empty())
            roadgraph.road_nodes_.erase(node->id);
            */
        if (node->connectNodes.size() > 2) {
            intersection_nodes.push_back(node);
        }
    }

    //如果两个交叉路口相邻，向中间插一个点
    for (auto intersection : intersection_nodes) {
        for (auto connectPoint : intersection->connectNodes) {
            if (connectPoint->connectNodes.size() > 2) {
                double     x = (intersection->localCoord.x + connectPoint->localCoord.x) / 2.0;
                double     y = (intersection->localCoord.y + connectPoint->localCoord.y) / 2.0;
                double     z = (intersection->localCoord.z + connectPoint->localCoord.z) / 2.0;
                VPE::dvec3 localCoord  = VPE::dvec3(x, y, z);
                auto       globalCoord = roadgraph.ToGlobal(localCoord);
                VPE::dvec3 lonLatAlt;
                g_coord.GlobalCoord2LongLat(globalCoord, lonLatAlt);
                RoadObjectId insertId    = id_alloc.AllocID();
                while (roadgraph.GetRoadNode(insertId) != nullptr) {
                    insertId = id_alloc.AllocID();
                }
                RoadNode *insertPoint   = new RoadNode(insertId, lonLatAlt);
                insertPoint->localCoord = localCoord;

                for (auto &t : intersection->connectNodes) {
                    if (t == connectPoint) {
                        t = insertPoint;
                        break;
                    }
                }

                for (auto &t : connectPoint->connectNodes) {
                    if (t == intersection) {
                        t = insertPoint;
                        break;
                    }
                }
                insertPoint->connectNodes.push_back(intersection);
                insertPoint->connectNodes.push_back(connectPoint);

                //重新改变拓扑关系，改变各个节点的connectNode
                for (auto id1 : intersection->edges) {
                    for (auto id2 : connectPoint->edges) {
                        if (id1 == id2) {
                            insertPoint->edges.push_back(id1);
                            break;
                        }
                    }
                }
                roadgraph.AddRoadNode(insertPoint);
            }
        }
    }
}

void RoadConstructor::OffsetLine(const std::vector<VPE::dvec3>& line, std::vector<VPE::dvec3>& offset_line, float offset)
{
	offset_line.clear();
	size_t pt_size = line.size();
	offset_line.reserve(pt_size);

	
	VPE::dvec3 last2next_dir;	//垂直于角平分线的向量	
	float real_offset;	//计算offset在角平分线方向的投影长度

	//首点
	VPE::dvec3 offset_dir = VPE::cross(VPE::normalize(line[1] - line[0]), VPE::dunit3_y);
	offset_line.push_back(line[0] + offset_dir * (double)offset);
	//中间点
	for (size_t i = 1, count = pt_size - 1; i < count; ++i)
	{
		size_t last = i - 1;
		size_t next = i + 1;

		//当前点指向上一个点的向量
		VPE::dvec3 cur2last_dir = VPE::normalize(line[last] - line[i]);
		//当前点指向下一个点的向量
		VPE::dvec3 cur2next_dir = VPE::normalize(line[next] - line[i]);
		//垂直于角平分线的向量
		VPE::dvec3 last2next_dir = VPE::normalize(cur2next_dir - cur2last_dir);
		//角平分线方向，即左右边界点偏移的向量
		VPE::dvec3 offset_dir = VPE::cross(last2next_dir, VPE::dunit3_y);

		//计算offset在角平分线方向的投影长度
		float cosine = VPE::dot(-VPE::normalize(cur2last_dir), last2next_dir);
		float real_offset = offset;
		if (cosine > 0.01)
			real_offset = offset / cosine;
		else 
			real_offset = offset;

		VPE::dvec3 offset_point = line[i] + offset_dir * (double)real_offset;
		offset_line.push_back(offset_point);
	}
	//末尾点
	offset_dir = VPE::cross(VPE::normalize(line[pt_size - 1] - line[pt_size - 2]), VPE::dunit3_y);
	offset_line.push_back(line[pt_size - 1] + offset_dir * (double)offset);
}


void RoadConstructor::ConstructMesh(RoadEdge& edge)
{
    //隧道不生成，反正看不见
    if (edge.is_tunnel_)
        return;

    edge.ClearMeshData();

    size_t nd_size = edge.global_points_.size();
    assert(nd_size > 1);

    float road_right_width = edge.width_ / 2.0f;
    float road_left_width  = edge.width_ / 2.0f;
    float len_right        = 0.0f; //累积right点的长度
    float len_left         = 0.0f; //累积left点的长度

    //根据车道及车向计算纹理坐标y值
    float coord_y_right = 0.0f;
    float coord_y_left  = 0.0f;
    if (edge.is_railway_) {
        coord_y_left = 1;
    }
    else if (edge.lane_nums_ > 1) {
        if (edge.is_oneway_) //单向道，白色中心标线
        {
            if (edge.lane_nums_ < 4)
                coord_y_right = 0.0f;
            else
                coord_y_right = float(24 - edge.lane_nums_) * 0.5 / 16.0f;
        } else //双向道，黄色中心标线
        {
            if (edge.lane_nums_ < 4)
                coord_y_right = float(16 - edge.lane_nums_) * 0.5 / 16.0f;
            else
                coord_y_right = float(8 - edge.lane_nums_) * 0.5f / 16.0f;
        }
        coord_y_left = coord_y_right + (edge.lane_nums_ / 16.0f) - 0.01;
        coord_y_right += 0.01;
    }

    VPE::dvec3 right(0.0), left(0.0);

    VPE::dvec3 up(0.0, 1.0, 0.0);

    edge.aabb_[0] = VPE::dvec3(DBL_MAX, DBL_MAX, DBL_MAX);
    edge.aabb_[1] = VPE::dvec3(-DBL_MAX, -DBL_MAX, -DBL_MAX);
    //// 高速公路加入应急车道
    // if (HasAttribute("emergency_lane_width"))
    //{
    //	road_right_width += GetAttribute<float>("emergency_lane_width");
    //}

    //保存道路边缘的网格
    RoadMesh *road_surface    = new RoadMesh; //路面
    RoadMesh *curb_mesh_right = new RoadMesh;
    RoadMesh *curb_mesh_left  = new RoadMesh;
    road_surface->tex_name    =  edge.is_railway_ ? "railway" : "asphalt";
    curb_mesh_left->tex_name  = "curb";
    curb_mesh_right->tex_name = "curb";

    for (size_t i = 0; i < nd_size; ++i) {
        size_t last = i > 0 ? i - 1 : 0;
        size_t next = i < (nd_size - 1) ? i + 1 : (nd_size - 1);

        double     cosine  = 1.0;
        VPE::dvec3 dirNext = VPE::normalize(edge.global_points_[next] - edge.global_points_[last]);
        if (i > 0) {
            VPE::dvec3 dir = VPE::normalize(edge.global_points_[i] - edge.global_points_[last]);
        }
        VPE::dvec3 split_dir = VPE::normalize(VPE::cross(edge.global_points_[i], dirNext));

        right = edge.global_points_[i] + split_dir * double(road_right_width) / cosine;
        left  = edge.global_points_[i] - split_dir * double(road_left_width) / cosine;

        RoadVertex right_vertex;

        right_vertex.pos    = edge.road_graph_->ToLocal(right);
        right_vertex.normal = up;
        road_surface->vertices.emplace_back(right_vertex);
        len_right += VPE::distance(right_vertex.pos, road_surface->vertices[2 * last].pos);
        road_surface->vertices[2 * i].tex_coords =
            VPE::fvec2(len_right / edge.width_ * (float(edge.lane_nums_) / 2.0f), coord_y_right);

        RoadVertex left_vertex;
        left_vertex.pos    = edge.road_graph_->ToLocal(left);
        left_vertex.normal = up;
        road_surface->vertices.emplace_back(left_vertex);
        len_left += VPE::distance(left_vertex.pos, road_surface->vertices[2 * last + 1].pos);
        road_surface->vertices[2 * i + 1].tex_coords =
            VPE::fvec2(len_left / edge.width_ * (float(edge.lane_nums_) / 2.0f), coord_y_left);

        edge.UpdateBBox(right_vertex.pos);
        edge.UpdateBBox(left_vertex.pos);

        if (i > 0) {
            //建立索引
            int i1 = 2 * (i - 1);

            road_surface->indexes.emplace_back(i1);
            road_surface->indexes.emplace_back(i1 + 1);
            road_surface->indexes.emplace_back(i1 + 3);

            road_surface->indexes.emplace_back(i1);
            road_surface->indexes.emplace_back(i1 + 3);
            road_surface->indexes.emplace_back(i1 + 2);
        }

        //产生道路边缘
        if (edge.is_bridge_) {
            CalcCurbs(*curb_mesh_right, right_vertex.pos, up, split_dir, len_right);
            CalcCurbs(*curb_mesh_left, left_vertex.pos, up, -split_dir, len_left);
        }
    }
    edge.road_mesh_.emplace_back(road_surface);

    //合并mesh
    if (edge.is_bridge_) {
        edge.road_mesh_.emplace_back(curb_mesh_left);
        edge.road_mesh_.emplace_back(curb_mesh_right);
    } else {
        delete curb_mesh_left;
        delete curb_mesh_right;
    }

    //生成路灯
    if (edge.has_light_)
        CalcRoadLightData(edge, road_left_width, road_right_width);
    if (edge.is_bridge_)
        CalcBridgeColumn(edge, road_left_width, road_right_width);
}


void RoadConstructor::CalcCurbs(RoadMesh &curb_mesh, const VPE::dvec3 &pt, const VPE::dvec3 &up,
                         const VPE::dvec3 &split_dir, float len) {
    //建立curb顶点
    static bool                  reverse    = false; //两边的顺序不一样，需要翻转一下
    static double                curb_up    = 0.2;
    static double                curb_down  = -1.0;
    static double                curb_width = 0.5;
    static std::array<double, 4> vertical   = {curb_down, curb_up, curb_up, curb_down};
    std::array<double, 4>        horizontal;
    std::array<VPE::dvec3, 4>    normals;

    if (reverse) {
        horizontal = {0, 0, curb_width, curb_width};
        normals    = {-split_dir, VPE::normalize(-split_dir + up), VPE::normalize(split_dir + up),
                   split_dir};
    } else {
        horizontal = {curb_width, curb_width, 0, 0};
        normals    = {split_dir, VPE::normalize(split_dir + up), VPE::normalize(-split_dir + up),
                   -split_dir};
    }

    for (int i = 0; i < 4; i++) {
        RoadVertex vertex;
        vertex.pos        = pt + split_dir * horizontal[i] + up * vertical[i];
        vertex.normal     = normals[i];
        vertex.tex_coords = VPE::fvec2(len * 0.25, 1.0 / float(i));
        curb_mesh.vertices.emplace_back(vertex);
    }

    size_t vert_size = curb_mesh.vertices.size();
    int    i         = (vert_size - 4) / 4;
    int    i1        = i - 1;

    if (vert_size > 4) //建立索引
    {
        for (int a = 0; a < 3; ++a) {
            //三角面1
            curb_mesh.indexes.emplace_back(i1 * 4 + a);
            curb_mesh.indexes.emplace_back(i1 * 4 + a + 1);
            curb_mesh.indexes.emplace_back(i * 4 + a);

            //三角面2
            curb_mesh.indexes.emplace_back(i1 * 4 + a + 1);
            curb_mesh.indexes.emplace_back(i * 4 + a + 1);
            curb_mesh.indexes.emplace_back(i * 4 + a);
        }
    }

    reverse = !reverse;
}

void RoadConstructor::CalcRoadLightData(RoadEdge &edge, float road_left_width,
                                        float road_right_width) {
    double temp = 0.0;
    for (size_t i = 1; i < edge.global_points_.size(); ++i) {
        const VPE::dvec3 &start = edge.global_points_[i - 1];
        const VPE::dvec3 &end   = edge.global_points_[i];

        double dist = VPE::distance(end, start);

        double light_offset_to_road = edge.is_bridge_ ? -2 : 1;

        VPE::dvec3 dir   = VPE::normalize(end - start);
        VPE::dvec3 up    = VPE::normalize(start);
        VPE::dvec3 right = VPE::normalize(VPE::cross(dir, start));

        VPE::dvec3 light_pos, light_up, light_right, light_dir;
        VPE::dvec3 glm_pos, glm_right;

        double model_scale = g_roadgraph_interface.GetStreetLightModelParam()[0].scale;

        VPE::dmat4 scale(VPE::identity<VPE::dmat4>());
        VPE::set_scale(scale, VPE::dvec3(model_scale, model_scale, model_scale));

        while (true) {
            if (dist - temp < 0.0) {
                temp = temp - dist;
                break;
            }

            // 右侧路灯
            glm_pos   = start + dir * temp + right * (road_right_width + light_offset_to_road);
            glm_right = right;

            light_up    = VPE::dvec3(glm_pos.x, glm_pos.y, glm_pos.z);
            light_right = VPE::dvec3(right.x, right.y, right.z);
            light_pos   = light_up;

            light_up  = VPE::normalize(light_up);
            light_dir = VPE::cross(light_up, light_right);
            light_dir = VPE::normalize(light_dir);

            VPE::dmat4 mat =
                RoadUtils::BuildLocalTransform(light_right, light_up, light_dir, light_pos) * scale;
            edge.road_graph_->AddLight(mat);

            //左侧路灯
            if (!edge.is_oneway_) {
                glm_pos   = start + dir * temp - right * (road_left_width + light_offset_to_road);
                glm_right = -right;

                light_up    = VPE::dvec3(glm_pos.x, glm_pos.y, glm_pos.z);
                light_right = VPE::dvec3(glm_right.x, glm_right.y, glm_right.z);
                light_pos   = light_up;

                light_up  = VPE::normalize(light_up);
                light_dir = VPE::cross(light_up, light_right);
                light_dir = VPE::normalize(light_dir);

                VPE::dmat4 mat =
                    RoadUtils::BuildLocalTransform(light_right, light_up, light_dir, light_pos) *
                    scale;
                edge.road_graph_->AddLight(mat);
            }

            temp += edge.light_space_;
        }
    }
}


void RoadConstructor::CalcBridgeColumn(RoadEdge &edge, float road_left_width,
                                       float road_right_width) {
    double temp = 0.0;

    for (size_t i = 1; i < edge.global_points_.size(); ++i) {
        const VPE::dvec3 &start = edge.global_points_[i - 1];
        const VPE::dvec3 &end   = edge.global_points_[i];

        double dist = VPE::distance(end, start);

        double column_offset_to_road = -4;

        VPE::dvec3 dir   = VPE::normalize(end - start);
        VPE::dvec3 up    = VPE::normalize(start);
        VPE::dvec3 right = VPE::normalize(VPE::cross(dir, start));

        VPE::dvec3 col_pos, col_up, col_right, col_dir;
        VPE::dvec3 glm_pos, glm_right;

        while (true) {
            if (dist - temp < 0.0) {
                temp = temp - dist;
                break;
            }

            col_pos = start + dir * temp;

            col_up    = col_pos;
            col_right = right;

            col_up  = VPE::normalize(col_up);
            col_dir = VPE::cross(col_up, col_right);
            col_dir = VPE::normalize(col_dir);

            VPE::dmat4 mat = RoadUtils::BuildLocalTransform(col_right, col_up, col_dir, col_pos);
            edge.road_graph_->AddColumn(mat);            
            temp += edge.column_space_;
        }
    }
}


//
//void RoadConstructor::ConstructLanes(RoadGraph* roadgraph)
//{
//	for (auto& edge_item : roadgraph->road_edges_)
//	{
//		auto edge = edge_item.second;
//		int lane_num = edge->lane_nums_;
//
//		for (int i = 0; i < lane_num; ++i)
//		{
//			float right_offset = ((lane_num - 1) * 0.5f - i) * edge->lane_width_;
//			Lane* lane = new Lane;
//			edge->lanes_.push_back(lane);
//			OffsetLine(edge->local_points_, lane->points_, right_offset);
//		}
//	}
//}

void RoadConstructor::ConstructIntersection(RoadGraph& roadgraph)
{
    for (auto intersection : intersection_nodes) {
        RoadIntersection *node = new RoadIntersection(3.5, intersection);
        node->generatePoints();
        Intersections.push_back(node);
    }
}

const std::vector<VPE::dvec2> &RoadConstructor::ConstructArcRoads(RoadEdge &edge) {
    auto &       edge_nodes = edge.nodes_;
    RoadObjectId wayId      = edge.GetID();
    ArcRoad *    road       = new ArcRoad(&edge);
    arc_roads.push_back(road);
    return road->generatePoints();
}

void RoadConstructor::Construct3D(RoadEdge &edge, const std::vector<VPE::dvec2> &points2d) {

    //对全局坐标设置海拔高度
    auto SetAltitude = [](VPE::dvec3 &globalPos, double alt) {
        VPE::dvec3 lonlat;
        g_coord.GlobalCoord2LongLat(globalPos, lonlat);
        lonlat.z = alt;
        g_coord.LongLat2GlobalCoord(lonlat, globalPos);
    };

    	//插入必要的中间点以避免路面在地面以下
    

    //查找起始、结束节点的最小layer
    int       startLayer = INT_MAX, endLayer = INT_MAX;
    RoadNode *startNode = edge.nodes_.front();
    RoadNode *endNode   = edge.nodes_.back();
    for (auto e : startNode->edges) {
        startLayer = (std::min)(startLayer, e->layer);
    }
    for (auto e : endNode->edges) {
        endLayer = (std::min)(endLayer, e->layer);
    }

    //两头无layer，中间在高点
    if (startLayer == 0 && endLayer == 0 && edge.layer > 0) {

        double length = 0;
        for (auto i = 1; i < points2d.size(); ++i)
        {
            length += VPE::distance(points2d[i - 1], points2d[i]);
        }
        double half_len = length / 2.0;

        int midPointHeight = 10 + (edge.layer - 1) * 3;

        //三次样条方程
        //当addLength < length/2.0时
        double x1 = 0,
               y1 = ViWoROOT::GetTerrainInterface()->GetPreciseElevation(startNode->lonLatAlt.x,
                                                                         startNode->lonLatAlt.y) +
                    offset_to_terrain_,
               x2 = half_len, y2 = midPointHeight;
        double a = y1, b = 0, c = 3 * (y2 - y1) / ((x2 - x1) * (x2 - x1));
        double d = -2 * (y2 - y1) / ((x2 - x1) * (x2 - x1) * (x2 - x1));
        //当addLength > length/2.0时
        double m1 = 0, n1 = midPointHeight, m2 = length / 2.0,
               n2 = ViWoROOT::GetTerrainInterface()->GetPreciseElevation(endNode->lonLatAlt.x,
                                                                         endNode->lonLatAlt.y) +
                    offset_to_terrain_;
        double a1 = n1, b1 = 0, c1 = 3 * (n2 - n1) / ((m2 - m1) * (m2 - m1));
        double d1 = -2 * (n2 - n1) / ((m2 - m1) * (m2 - m1) * (m2 - m1));

        bool is_first_half = true; // 是否前半段

        double addLength    = 0;
        double addStep   = 5;

        for (auto i = 1; i < points2d.size(); ++i) {
            double gapLength    = VPE::distance(points2d[i-1], points2d[i]);
            double futureLength = addLength + gapLength;
            auto   n            = VPE::normalize(points2d[i] - points2d[i-1]);
            VPE::dvec3 lonlat;
            for (int dis = 0; dis < gapLength; dis += addStep) {
                if (addLength > half_len && is_first_half) {
                    is_first_half = false; //改为后半段
                }
                VPE::dvec2 localPos = points2d[i-1] + double(dis)*n;

                double insertHeight;
                if (is_first_half) {
                    insertHeight = a + c * (addLength - x1) * (addLength - x1) +
                                   d * (addLength - x1) * (addLength - x1) * (addLength - x1);
                } else {
                    insertHeight = a1 +
                                   c1 * (addLength - half_len - m1) * (addLength - half_len - m1) +
                                   d1 * (addLength - half_len - m1) * (addLength - half_len - m1) *
                                       (addLength - half_len - m1);
                }
                VPE::dvec3 globalPos = edge.road_graph_->ToGlobal(VPE::dvec3(localPos.x, 0, localPos.y));
                SetAltitude(globalPos, insertHeight);
                edge.global_points_.emplace_back(globalPos);
                addLength += addStep;
            }
            addLength = futureLength;
        } 
        endNode->alt         = n2;
        VPE::dvec3 globalPos;
        g_coord.LongLat2GlobalCoord(endNode->lonLatAlt, globalPos);
        edge.global_points_.emplace_back(globalPos);  
        return;
    }

    if (startLayer == 0 && endLayer == 0) {
        //如果是普通道路，贴地处理
        for (auto & point2d : points2d)
        {
            VPE::dvec3 global = edge.road_graph_->ToGlobal(VPE::dvec3(point2d.x, 0, point2d.y));
            VPE::dvec3 lonlat;
            g_coord.GlobalCoord2LongLat(global, lonlat);
            double el = ViWoROOT::GetTerrainInterface()->GetPreciseElevation(lonlat.x, lonlat.y);
            lonlat.z  = el + offset_to_terrain_;
            g_coord.LongLat2GlobalCoord(lonlat, global);
            edge.global_points_.push_back(global);
        }
        InsertPointsToGround(edge);
        return;
    }

    //都在同一高层
    if (startLayer == endLayer) {
        // double h = 50 + (startLayer - 1) * 10;
        double h = 10 + (startLayer - 1) * 3;
        for (int i = 0; i < points2d.size(); i++) {
            VPE::dvec3 globalPos = edge.road_graph_->ToGlobal(
                VPE::dvec3(points2d[i].x, 0, points2d[i].y));
            SetAltitude(globalPos, h);
            edge.global_points_.push_back(globalPos);
        }
        return;
    }

    //两头在不同层
    if (startLayer != endLayer) {

        double h1, h2;
        if (startLayer == 0) {
            h1 = ViWoROOT::GetTerrainInterface()->GetPreciseElevation(startNode->lonLatAlt.x,
                                                                      startNode->lonLatAlt.y) +
                 offset_to_terrain_;
        } else {
            if (10 + (startLayer - 1) * 3 > ViWoROOT::GetTerrainInterface()->GetPreciseElevation(
                                                startNode->lonLatAlt.x, startNode->lonLatAlt.y) +
                                                offset_to_terrain_) {
                // h1 = 50 + (startLayer - 1) * 10;
                h1 = 10 + (startLayer - 1) * 3;
            } else {
                h1 = ViWoROOT::GetTerrainInterface()->GetPreciseElevation(startNode->lonLatAlt.x,
                                                                          startNode->lonLatAlt.y) +
                     offset_to_terrain_;
            }
        }

        if (endLayer == 0) {
            h2 = ViWoROOT::GetTerrainInterface()->GetPreciseElevation(endNode->lonLatAlt.x,
                                                                      endNode->lonLatAlt.y) +
                 offset_to_terrain_;
        } else {
            if (10 + (endLayer - 1) * 3 > ViWoROOT::GetTerrainInterface()->GetPreciseElevation(
                                              endNode->lonLatAlt.x, endNode->lonLatAlt.y) +
                                              offset_to_terrain_) {
                h2 = 10 + (endLayer - 1) * 3;
            } else {
                h2 = ViWoROOT::GetTerrainInterface()->GetPreciseElevation(endNode->lonLatAlt.x,
                                                                          endNode->lonLatAlt.y) +
                     offset_to_terrain_;
            }
        }

        double length = 0;
        for (auto i = 1; i < points2d.size(); ++i) {
            length += VPE::distance(points2d[i - 1], points2d[i]);
        }
        double half_len = length / 2.0;
        //三次样条
        double x1 = 0, y1 = h1, x2 = length, y2 = h2;
        double a = y1, b = 0, c = 3 * (y2 - y1) / ((x2 - x1) * (x2 - x1));
        double d  = -2 * (y2 - y1) / ((x2 - x1) * (x2 - x1) * (x2 - x1));

        double addLength = 0;
        double addStep   = 5;
        for (auto i = 1; i < points2d.size(); ++i) {
            double gapLength    = VPE::distance(points2d[i - 1], points2d[i]);
            double futureLength = addLength + gapLength;
            auto   n            = VPE::normalize(points2d[i] - points2d[i - 1]);

            for (int dis = 0; dis < gapLength; dis += addStep) {

                VPE::dvec2 localPos = points2d[i-1] + (double)dis * n;

                double insertHeight = a + c * (addLength - x1) * (addLength - x1) +
                                      d * (addLength - x1) * (addLength - x1) * (addLength - x1);
                
                VPE::dvec3 globalPos =
                    edge.road_graph_->ToGlobal(VPE::dvec3(localPos.x, 0, localPos.y));
                SetAltitude(globalPos, insertHeight);
                edge.global_points_.emplace_back(globalPos);
                addLength += addStep;
            }
            addLength    = futureLength;
        }
        endNode->alt = h2;
        VPE::dvec3 globalPos;
        g_coord.LongLat2GlobalCoord(endNode->lonLatAlt, globalPos);
        edge.global_points_.emplace_back(globalPos); 
    }
}

void RoadConstructor::InsertPointsToGround(RoadEdge &  edge) {

    float offset_thres = offset_to_terrain_ * 0.5; //控制插入点的阈值
    float step         = 0.5;                      //迭代插入点的步长(单位米)

    auto itr = edge.global_points_.begin(); //起始点
    itr++;
    glm::dvec3 last_global, cur_global;
    last_global = edge.global_points_[0];
    for (; itr != edge.global_points_.end(); ++itr) {
        glm::dvec3 cur_global = *itr;
        double     dis        = glm::distance(last_global, cur_global);
        int        count      = dis / step;
        glm::dvec3 dir_step   = (cur_global - last_global) / double(count);
        glm::dvec3 tmp_point = last_global;
        for (int i = 1; i < count; ++i) {
            tmp_point = tmp_point + dir_step;
            glm::dvec3 tmp_lonlat;
            g_coord.GlobalCoord2LongLat(tmp_point, tmp_lonlat);
            double el =
                ViWoROOT::GetTerrainInterface()->GetPreciseElevation(tmp_lonlat.x, tmp_lonlat.y);
            if (tmp_lonlat.z < el + offset_thres) {
                tmp_lonlat.z = el + offset_to_terrain_;
                g_coord.LongLat2GlobalCoord(tmp_lonlat, tmp_point);
                dir_step = (cur_global - tmp_point) / double(count - i);
                itr      = edge.global_points_.insert(itr, tmp_point);
                ++itr;
            }
        }
        last_global = cur_global;
    }
}
    //void RoadConstructor::ShowLanes(RoadGraph* roadgraph)
//{
//	for (auto& edge_item : roadgraph->road_edges_)
//	{
//		for (auto lane : edge_item.second->lanes_)
//		{
//			static VPE::fvec4 color{ 1.0f, 1.0f, 0.0f, 1.0f };
//			ShowLocalLine(lane->points_, color, roadgraph);
//		}
//	}
//}
void RoadConstructor::ShowRoadNodes(RoadGraph &roadgraph) {
    static VPE::fvec4 node_color{1.0f, 0.0f, 1.0f, 1.0f};
    static VPE::fvec4 edge_color{0.0f, 0.0f, 1.0f, 1.0f};
    for (auto &edge_item : roadgraph.road_edges_) {
        std::vector<VPE::dvec3> global_points;
        for (auto node : edge_item.second->nodes_) {
            VPE::dvec3 global_point = roadgraph.ToGlobal(node->localCoord);
            vg_->DrawSpacePoint(global_point, node_color, 3.0f);
            global_points.push_back(global_point);
        }

        vg_->DrawSpaceLine(global_points, edge_color, 1.0f);
    }
}

void RoadConstructor::ShowIntersections(RoadGraph &roadgraph) {
    fvec4 color{1.0, 0.0, 0.0, 1.0};
    auto vg_render = ViWoROOT::GetVectorGraphicsRender();
    for (auto intersection : Intersections) {
        auto boundaries = intersection->boundaries;
        for (auto &edge : boundaries) {
            std::vector<VPE::dvec3> edge_points;
            for (int i = 0; i < edge.size(); i++) {
                glm::dvec3 localPosition(edge[i].x, 0, edge[i].y);
                glm::dvec3 globalPosition = roadgraph.ToGlobal(localPosition, true);
                edge_points.emplace_back(globalPosition.x, globalPosition.y, globalPosition.z);
            }            
            vg_render->DrawSpaceLine(edge_points, color, 1.0);
        }
    }
}