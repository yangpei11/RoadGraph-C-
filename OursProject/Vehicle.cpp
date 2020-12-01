
#include "Vehicle.h"
#include "RoadEdge.h"
#include "RoadNode.h"
#include "VehicleSchedule.h"
#include "RoadGraphInterface.h"


#include "MicroProfiling.h"

#include "ViWoRoot.h"
#include "Camera.h"
#include "CameraManager.h"

#include "World.h"
#include "Light.h"


#include <cassert>
#include <cstdint>

using namespace glm;

Vehicle::Vehicle(const std::string& model_name, double scale, VehicleSchedule* vehicle_scheduler)
	:cur_node_(nullptr),
	cur_edge_(nullptr),
	next_node_(nullptr),
	next_edge_(nullptr),
	cur_lane_number_(-1),
	//next_lane_number_(1),
	cur_point_id_(0),
	cur_len_(0.0f),
	cur_edge_len_(0.0f),
    model_name_(model_name),
    scale_(scale),
    cur_position_(nullptr),
    is_setcamera_(false),
    vehicle_scheduler_(vehicle_scheduler)
{

    last_time_ = 0.0;

    Hide();
	
}

void Vehicle::GenerateLights()
{
	VPE::World* world = ViWoROOT::World();
	if (model_name_.find("c5") == std::string::npos)
	{
		static VPE::fvec3 light_color_front(1, 1, 1);
		static VPE::fvec3 light_color_tail(1, 0, 0);
		static double glow_size = 0.1;
		static double min_range = 0.0;
		static double max_range_front = 30;
		static double max_range_tail = 30;

		auto             entity = world->CreateEntity();
        world->assign<VPE::Light>(entity, VPE::POINT_LIGHT, light_color_front);
        lights_.push_back(entity);

		entity = world->CreateEntity();
        world->assign<VPE::Light>(entity, VPE::POINT_LIGHT, light_color_front);
        lights_.push_back(entity);

		entity = world->CreateEntity();
        world->assign<VPE::Light>(entity, VPE::POINT_LIGHT, light_color_tail);
        lights_.push_back(entity);

		entity = world->CreateEntity();
        world->assign<VPE::Light>(entity, VPE::POINT_LIGHT, light_color_tail);
        lights_.push_back(entity);
	}
}

Vehicle::~Vehicle()
{
    //SceneEditTool::Instance()->DeleteModel(scenegraph_id_, model_id_);
}

void Vehicle::InitParam(std::uint64_t * seed)
{
	cur_position_ = cur_edge_;
    int lane_num     = cur_edge_->lane_nums_;
	cur_lane_number_ = vehicle_scheduler_->GetLaneNumberSequence(cur_lane_number_, *seed);
	speed_ = (*seed)%cur_edge_->max_speed_;
	right_offset_ = ((lane_num - 1) * 0.5 - cur_lane_number_) * cur_edge_->lane_width_ + 1;
	next_node_ = cur_edge_->GetOtherNode(cur_node_);
	next_edge_ =  next_node_ ? next_node_->edges[(*seed)%(next_node_->edges.size())] : nullptr;
}

void Vehicle::UpdateParam()
{
	auto &cur_point = cur_edge_->global_points_[cur_point_id_];
    auto &next_point = cur_edge_->global_points_[next_point_id_];
	cur_edge_len_ = VPE::distance(cur_point, next_point);
	cur_dir_ = VPE::normalize(next_point - cur_point);
	cur_right_ = VPE::normalize(VPE::cross(cur_dir_, cur_point));
}

void Vehicle::Update(double cur_time, const VPE::dvec3& camera_pos)
{
    if(is_hided_) 
		return;

	//如果所处当前位置已不可见，回收
	if (!vehicle_scheduler_->IsVisible(cur_position_)) 
	{
		Recycle();
		return;
	}

    // 下面的代码是计算在时间间隔内车辆应该运动的距离
    double time_interval = cur_time - last_time_;
    double dist = time_interval * speed_;	//两帧之间移动距离
    float len_sum = cur_len_ + dist;	

    last_time_ = cur_time;

    std::uint64_t* seed = reinterpret_cast<std::uint64_t*>(&cur_time);

    double cur_turn_length;

	auto TurntoNextEdge = [this, &len_sum, &seed]()->bool
	{
        if ((cur_node_ == cur_edge_->nodes_.front() &&
             cur_point_id_ == cur_edge_->global_points_.size() - 2) ||
			(cur_node_ == cur_edge_->nodes_.back() && cur_point_id_ == 1))
		{
			/*speed_ = next_node_->GetTurnRegularSpeed();
			cur_position_ = next_node_;*/
			if (next_edge_)
			{
				cur_edge_ = next_edge_;
				cur_node_ = next_node_;
				cur_len_ = len_sum - cur_edge_len_; //如果时间间隔较大，点距较小，会有潜在问题
				InitParam(seed);

				if (cur_node_ == cur_edge_->nodes_.front())
				{
					cur_point_id_ = 0;
					next_point_id_ = cur_point_id_ + 1;
				}
				else
				{
                    cur_point_id_  = cur_edge_->global_points_.size() - 1;
					next_point_id_ = cur_point_id_ - 1;
				}

				UpdateParam();

			}
			else //没有下一条边了，回收
			{
				Recycle();
			}
			return true;
		}
		return false;
	};


    // 下面的代码根据车辆的运动的距离，计算车辆应该处于路网的哪一个对象中
    if(cur_position_ != nullptr)
    {
		//简化问题，暂时不考虑node的连接处情况
        if (cur_position_ == cur_edge_)
        {
            if (len_sum >= cur_edge_len_)//驶过一小段边
            {
				//未驶向下一条边，转过一小段边
				if (!TurntoNextEdge())
				{
					while (true)
					{
						if (cur_node_ == cur_edge_->nodes_.front())
						{
							cur_point_id_++;
							next_point_id_ = cur_point_id_ + 1;
						}
						else
						{
							cur_point_id_--;
							next_point_id_ = cur_point_id_ - 1;
						}

						cur_len_ = len_sum - cur_edge_len_;
						len_sum = cur_len_;
						UpdateParam();
						if(cur_len_ < cur_edge_len_)
							break;

						if (TurntoNextEdge())
						{
							return;
						}
					}
					
				}
            }
			else //还在当前的一段边上
			{
				cur_len_ = len_sum; //仅更新当前驶过的距离
			}

        }
    }// end of while


    //  下面的代码是根据计算得到当前车辆所处的道路对象计算他的 位置（pos）和方向（dir）
	dvec3 pos, normal;
    if ( cur_position_ != nullptr)
    {
		//暂时不考虑位于node情况
        if (cur_position_ == cur_edge_)
        {
            const VPE::dvec3 &cur_pos = cur_edge_->global_points_[cur_point_id_];
			
			
			pos = cur_pos + cur_dir_ * double(cur_len_) + cur_right_ * double(right_offset_);
        }
        //else
        //{
        //    /*assert(cur_edge_->IsNode(next_node_));
        //    assert(next_edge_->IsNode(next_node_));*/

        //    pos = next_node_->GetPosition(cur_edge_, next_edge_, cur_lane_number_, next_lane_number_, last_offset_);

        //    dir = next_node_->GetDirection(cur_edge_, next_edge_, cur_lane_number_, next_lane_number_, last_offset_);
        //}

        // 下面的代码是根据计算出来的pos和dir计算模型的变换矩阵

        normal = VPE::normalize(pos);

        if (is_setcamera_)
        {
            SetCamera(pos, normal, cur_dir_);
        }

        if (VPE::distance(pos, camera_pos) > g_roadgraph_interface.GetStartShowVehicleDistance())
        {
            Recycle(); //跑太远了，回收
        }

        VPE::dmat4 scale(VPE::identity<VPE::dmat4>());
        VPE::set_scale(scale, VPE::dvec3(scale_, scale_, scale_));

        VPE::dmat4 local(
			cur_right_.x, cur_right_.y, cur_right_.z, 0.0,
            normal.x, normal.y, normal.z, 0.0,
            cur_dir_.x, cur_dir_.y, cur_dir_.z, 0.0,
            pos.x, pos.y, pos.z, 1.0
            );

        VPE::dmat4 mat =  local * scale;
        vehicle_scheduler_->PushRenderVehicle(model_name_, mat);
		UpdateLights(mat);
    }
}

void Vehicle::UpdateLights(const VPE::dmat4& mat)
{

	auto world = ViWoROOT::World();
	if ( !lights_.empty())
	{
		VPE::dvec3 light_direction;
        light_direction = cur_dir_;
		static VPE::dvec3 front_left(-0.876, 0.535, 1.728);
		static VPE::dvec3 front_right(0.876, 0.535, 1.728);
		static VPE::dvec3 tail_left(0.8, 0.56, -2.15);
		static VPE::dvec3 tail_right(-0.8, 0.56, -2.15);

		/// front left
		VPE::dvec3 light_position = VPE::trans_pos(mat, front_left);
        world->SetPosition(lights_[0], light_position);

		/// front right
		light_position = VPE::trans_pos(mat, front_right);
        world->SetPosition(lights_[1], light_position);

		/// tail left
        light_position = VPE::trans_pos(mat, tail_left);
        world->SetPosition(lights_[2], light_position);

		/// tail right
        light_position = VPE::trans_pos(mat, tail_right);
        world->SetPosition(lights_[3], light_position);
	}
}
void Vehicle::Hide()
{
    is_hided_ = true;
}

void Vehicle::Show()
{
    is_hided_ = false;
}

void Vehicle::Recycle()
{
	is_hided_ = true;
	cur_position_ = nullptr;
	cur_lane_number_ = -1;
}

void Vehicle::Start(double start_time, const RoadEdge* edge)
{
    assert(feature != nullptr);

    std::uint64_t* seed = reinterpret_cast<std::uint64_t*>(&start_time);

	//目前暂时只考虑Edge

    cur_edge_ = edge;
    if (cur_edge_)
    {
		cur_node_ = cur_edge_->is_oneway_ ? cur_edge_->nodes_.front() : cur_edge_->nodes_[(*seed) % 1 *(cur_edge_->nodes_.size()-1)];
        last_time_ = start_time;
		InitParam(seed);
        if (cur_edge_->global_points_.size() == 1)
            cur_point_id_ = 0;
        else
			cur_point_id_ =
            (*seed) % (cur_edge_->global_points_.size() - 1); //随机选一个中间点作为起始点

		if (cur_node_ == cur_edge_->nodes_.front())
		{
			next_point_id_ = cur_point_id_ + 1;
		}
		else
		{
			next_point_id_ = cur_point_id_ - 1;
		}
		UpdateParam();

        Show();
    }
    else
    {
        Hide();
        cur_node_ = nullptr;
    }
}

bool Vehicle::IsStopped() const
{
    return (cur_position_ == nullptr || is_hided_);
}

void Vehicle::Reset()
{
    last_time_ = 0.0;
    cur_node_ = nullptr;
    cur_position_ = nullptr;
}

void Vehicle::SetCamera(const VPE::dvec3& pos, const VPE::dvec3& up, const VPE::dvec3& dir)
{
    auto camera_p = ViWoROOT::GetCameraManager()->GetCurrCamera();

    VPE::dvec3 pos_v(0.0), up_v(0.0), dir_v(0.0), center(0.0);

    std::copy(value_ptr<double>(pos), value_ptr<double>(pos) + 3, value_ptr(pos_v));
    std::copy(value_ptr<double>(up), value_ptr<double>(up) + 3, value_ptr(up_v));
    std::copy(value_ptr<double>(dir), value_ptr<double>(dir) + 3, value_ptr(dir_v));

    if (camera_p)
    {
        center = pos_v + dir_v * 100.0;
        pos_v = pos_v + up_v * 1.0;

        camera_p->LookAt(pos_v, center, up_v);
    }
}

