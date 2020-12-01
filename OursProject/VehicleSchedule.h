#pragma once

#include "VPMath.h"
#include "CommonRoadType.h"
#include "VPMath.h"


#include <string>
#include <vector>
#include <list>

class Vehicle;
class RoadGraph;
class RoadIntersectionPoint;
class RoadFeature;

class VehicleSchedule
{
public:
    VehicleSchedule(int vehicle_num, const std::vector<ModelType>& models);

    ~VehicleSchedule();

    void Init(RoadGraph* road_graph);

    void Start();

    void Pause();

    void Stop();

    void Render();

    void UpdateTime(const VPE::dvec3& camera_pos, double cur_time);

    void HideAllVehicle() const;

    void ShowAllVehicle() const;

    void PushRenderVehicle(const std::string& name, const VPE::dmat4& mat)
    {
        auto it = runing_vehicle_parameter_.find(name);
        if (it != runing_vehicle_parameter_.end())
        {
            it->second.push_back(mat);
        }
        else
        {
            runing_vehicle_parameter_.insert(std::make_pair(name, std::vector<VPE::dmat4x4>(1, mat)));
        }
    }


    void SetVehicleNum(int num) { vehicle_num_ = num; }

    void AddModels(const std::vector<ModelType>& models)
    {
        models_.insert(models_.end(), models.begin(), models.end());
    }


	//判断feature是否可见
	bool IsVisible(const RoadFeature * const ft) const;

        /// \warning , 详细看代码
    int GetLaneNumberSequence(int last_lane_number,
                              const std::uint64_t &seed) const;

private:
    enum ScheduleState
    {
        S_Start,
        S_Pause,
        S_Stop
    };

    void StopAllVehicle();

    void FireVehicle(double cur_time);

    void GenerateVehicle();

    RoadGraph* road_graph_;

    std::vector<Vehicle*> stopped_vehicles_;
    std::list<Vehicle*> running_vehicles_;

    ScheduleState state_;

    std::vector<ModelType> models_;

    std::map< std::string, std::vector<VPE::dmat4x4> >  runing_vehicle_parameter_;

    int vehicle_num_;

    double last_fire_time_;
    double fire_interval_time;
};