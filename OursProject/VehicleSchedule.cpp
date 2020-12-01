#include "VehicleSchedule.h"
#include "Vehicle.h"
#include "RoadGraph.h"
#include "RoadEdge.h"
#include "RoadGraphInterface.h"
#include "VPMath.h"

#include "boost/asio.hpp"
#include "boost/serialization/vector.hpp"
#include "boost/archive/binary_iarchive.hpp"
#include "boost/archive/binary_oarchive.hpp"
#include "boost/serialization/vector.hpp"

#include "RenderTree.h"

#include <fstream>
#include "EffectManagerInterface.h"
#include "ViWoRoot.h"

#include "Light.h"


using namespace std;
using namespace VPE;

VehicleSchedule::VehicleSchedule(int vehicle_num, const std::vector<ModelType>& models)
    :vehicle_num_(vehicle_num),
    state_(S_Stop)
{
    AddModels(models);
}

VehicleSchedule::~VehicleSchedule()
{
    for_each(stopped_vehicles_.begin(), stopped_vehicles_.end(), [](Vehicle* v) { delete v; });
    for_each(running_vehicles_.begin(), running_vehicles_.end(), [](Vehicle* v) { delete v; });
}

void VehicleSchedule::Init(RoadGraph* road_graph)
{
    assert(road_graph != nullptr);

    state_ = S_Stop;

    road_graph_ = road_graph;
    assert(road_graph_ != nullptr);

    last_fire_time_ = 0.0;
    fire_interval_time = 3.0;

    GenerateVehicle();


}

void VehicleSchedule::Start()
{
    state_ = S_Start;
}

void VehicleSchedule::Pause()
{
    state_ = S_Pause;
}

void VehicleSchedule::Stop()
{
    state_ = S_Stop;

    StopAllVehicle();
}

void VehicleSchedule::Render()
{
    if (state_ == S_Stop) return;
    StaticRenderer::RenderTree::GetInstance()->AddInstance(runing_vehicle_parameter_);
}

void VehicleSchedule::UpdateTime(const VPE::dvec3& camera_pos, double cur_time)
{
    if (state_ == S_Stop)
    {
        return;
    }

    assert(road_graph_ != nullptr);

    runing_vehicle_parameter_.clear();

    if (state_ == S_Start)
    {
        FireVehicle(cur_time);
		for (auto it = running_vehicles_.begin(); it != running_vehicles_.end();)
		{
			if ((*it)->IsStopped())
			{
				stopped_vehicles_.push_back(*it);
				it = running_vehicles_.erase(it);
			}
			else
			{
				(*it)->Update(cur_time, camera_pos);
				++it;
			}
		}
    }
}

void VehicleSchedule::HideAllVehicle() const
{
    for (auto it = running_vehicles_.begin(); it != running_vehicles_.end(); ++it)
    {
        (*it)->Hide();
    }
}

void VehicleSchedule::ShowAllVehicle() const
{
    for (auto it = stopped_vehicles_.begin(); it != stopped_vehicles_.end(); ++it)
    {
        (*it)->Show();
    }
}

void VehicleSchedule::StopAllVehicle()
{
    for (auto it = running_vehicles_.begin(); it != running_vehicles_.end(); ++it)
    {
        (*it)->Reset();
        (*it)->Hide();
    }
}

void VehicleSchedule::FireVehicle(double cur_time)
{
    if (stopped_vehicles_.empty())	return;

    auto visible_features = road_graph_->GetVisibleFeatures();

    if (visible_features.empty())	return;

    std::uint64_t* seed = reinterpret_cast<std::uint64_t*>(&cur_time);

    size_t num = visible_features.size();

    if (cur_time - last_fire_time_ > fire_interval_time)
    {
        while (true)
        {
			int rad = rand() % num;
			auto itr = visible_features.begin();
			std::advance(itr, rad);
			RoadObjectId fid = *(itr);

			if (stopped_vehicles_.empty())	
				return;
            if (road_graph_->GetRoadEdge(fid)->is_railway_)
                continue;

            auto vehicle = stopped_vehicles_.back();

			vehicle->Start(cur_time, road_graph_->GetRoadEdge(fid));

            running_vehicles_.push_back(vehicle);
            stopped_vehicles_.pop_back();
        }

        last_fire_time_ = cur_time;
    }
}

void VehicleSchedule::GenerateVehicle()
{
    if (models_.empty())	return;

    int temp = vehicle_num_;

    while (temp)
    {
        for (size_t i = 0; i < models_.size(); ++i)
        {
            Vehicle* vh = new Vehicle(models_[i].name, models_[i].scale, this);
            stopped_vehicles_.push_back(vh);

			
            if (--temp == 0)
            {
                break;
            }
        }
    }
	
	//通过AddIncetance触发读取模型文件
	for (auto & model : models_)
	{
		PushRenderVehicle(model.name, VPE::identity<VPE::dmat4>());
	}
	StaticRenderer::RenderTree::GetInstance()->AddInstance(runing_vehicle_parameter_);


    if (g_roadgraph_interface.GetControlCameraVehicleNum() > 0 && g_roadgraph_interface.GetControlCameraVehicleNum() <= stopped_vehicles_.size())
    {
        auto stopped_vehicles_it = stopped_vehicles_.begin();
        std::advance(stopped_vehicles_it, g_roadgraph_interface.GetControlCameraVehicleNum() - 1);

        (*stopped_vehicles_it)->IsSetCamera(true);
    }

}


bool VehicleSchedule::IsVisible(const RoadFeature * const ft) const
{
	return road_graph_->IsVisible(ft);
}

int VehicleSchedule::GetLaneNumberSequence(int last_lane_number,
                                    const std::uint64_t &seed) const {
    int total_oneway_number = 0;

    if (total_oneway_number == 0)
        return total_oneway_number;

    if (last_lane_number == -1 || last_lane_number > total_oneway_number) {
        return seed % total_oneway_number;
    } else {
        return last_lane_number;
    }
}