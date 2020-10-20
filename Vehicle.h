
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

    /// ��Ҫ����ת�ӵ����
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
	void InitParam(std::uint64_t * seed);	//��ʼ����Ҫ�Ĳ���
	void UpdateParam();		//������Ҫ�Ĳ���
    void GenerateLights();  //��������
    void UpdateLights(const VPE::dmat4 & mat);    //���³���
    std::string model_name_;

    bool is_hided_;

    /// ��λ m/s
    float speed_;

    /// ��λΪ��
    double last_time_;

    double scale_;

    bool is_setcamera_;

    VehicleSchedule* vehicle_scheduler_;

	const RoadNode* cur_node_;
	const RoadEdge* cur_edge_;
    const RoadNode *next_node_;
	const RoadEdge* next_edge_;

	const RoadFeature* cur_position_;

	int cur_lane_number_; //��ǰ��������0��ʼ��������������
	//int next_lane_number_;

	size_t cur_point_id_; //RoadEdge��ĵ�����
	size_t next_point_id_; //��һ��������
	float cur_len_; //��ǰ��һС�α�����ʻ���ľ���
	float cur_edge_len_; //��ǰһС�αߵĳ���
	VPE::dvec3 cur_dir_; //��ǰ����
	VPE::dvec3 cur_right_; //��ǰ�Ҳ෽��
	float right_offset_; //��������ƫ����

    std::vector<VPE::Entity> lights_;
};
a