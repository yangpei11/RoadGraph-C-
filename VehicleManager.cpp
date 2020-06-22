#include "VehicleManager.h"
#include "RoadGraph.h"
#include "VehicleSchedule.h"

VehicleManager* VehicleManager::vehicle_manager_instance_ = nullptr;
std::mutex VehicleManager::instance_mutex_;

VehicleManager* VehicleManager::GetInstancePtr()
{
    if (vehicle_manager_instance_ == nullptr)
    {
        std::lock_guard<std::mutex> _lock(instance_mutex_);
        if (vehicle_manager_instance_ == nullptr)
        {
            vehicle_manager_instance_ = new VehicleManager;
        }
    }

    return vehicle_manager_instance_;
}

VehicleManager::VehicleManager()
{
    vehicle_schedulers_.clear();
}

void VehicleManager::Start()
{
    for (auto& scheduler : vehicle_schedulers_)
    {
        scheduler->Start();
    }
}

void VehicleManager::Render()
{
    for (auto& scheduler : vehicle_schedulers_)
    {
        scheduler->Render();
    }
}

void VehicleManager::UpdateTime(const VPE::dvec3& camera_pos, double cur_time)
{
    for (auto& scheduler : vehicle_schedulers_)
    {
        scheduler->UpdateTime(camera_pos, cur_time);
    }
}

void VehicleManager::AddScheduler(VehicleSchedule* vehicle_scheduler, RoadGraph* road_graph)
{
    vehicle_scheduler->Init(road_graph);

    vehicle_schedulers_.push_back(vehicle_scheduler);
}