#pragma once
#ifndef _VEHICLE_MANAGER_H_
#define _VEHICLE_MANAGER_H_

#include <mutex>
#include <vector>
#include "VPMath.h"

class RoadGraph;
class VehicleSchedule;

class VehicleManager
{
public:
    static VehicleManager* GetInstancePtr();

    void Start();

    void Render();

    void UpdateTime(const VPE::dvec3& camera_pos, double cur_time);

    void AddScheduler(VehicleSchedule* vehicle_scheduler, RoadGraph* road_graph);

private:
    VehicleManager();
    VehicleManager(const VehicleManager&) = delete;
    VehicleManager& operator = (const VehicleManager&) = delete;

private:
    static VehicleManager* vehicle_manager_instance_;

    static std::mutex instance_mutex_;

    std::vector<VehicleSchedule*> vehicle_schedulers_;
};
#endif // _VEHICLE_MANAGER_H_