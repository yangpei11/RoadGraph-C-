
#pragma once

#include <vector>
#include <string>
#include "VPMath.h"
#include "Entity.h"

class RoadFeature;
class RoadIntersectionPoint;
class RoadEdge;
class RoadNode;
class VehicleSchedule;

class Vehicle
{
public:
    Vehicle(const std::string& model_name, double scale, VehicleSchedule* vehicle_scheduler);

    virtual ~Vehicle();

    /// 需要处理转钟的情况
    virtual void Update(double cur_time, const VPE::dvec3& camera_pos);

    void Hide();

    void Show();

	void Recycle();

    void SetSpeed(double speed) { speed_ = speed; }

    void Start(double start_time, const RoadEdge* edge);

    bool IsStopped() const;

    void Reset();

    void IsSetCamera(bool is_set) { is_setcamera_ = is_set; }

protected:
    void SetCamera(const VPE::dvec3& pos, const VPE::dvec3& up, const VPE::dvec3& dir);
	void InitParam(std::uint64_t * seed);	//初始化需要的参数
	void UpdateParam();		//更新需要的参数
    void GenerateLights();  //产生车灯
    void UpdateLights(const VPE::dmat4 & mat);    //更新车灯
    std::string model_name_;

    bool is_hided_;

    /// 单位 m/s
    float speed_;

    /// 单位为秒
    double last_time_;

    double scale_;

    bool is_setcamera_;

    VehicleSchedule* vehicle_scheduler_;

	const RoadNode* cur_node_;
	const RoadEdge* cur_edge_;
    const RoadNode *next_node_;
	const RoadEdge* next_edge_;

	const RoadFeature* cur_position_;

	int cur_lane_number_; //当前车道，从0开始，自右向左增大
	//int next_lane_number_;

	size_t cur_point_id_; //RoadEdge里的点索引
	size_t next_point_id_; //下一个点索引
	float cur_len_; //当前在一小段边上行驶过的距离
	float cur_edge_len_; //当前一小段边的长度
	VPE::dvec3 cur_dir_; //当前朝向
	VPE::dvec3 cur_right_; //当前右侧方向
	float right_offset_; //车道所在偏移量

    std::vector<VPE::Entity> lights_;
};
a