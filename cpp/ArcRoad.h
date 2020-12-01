#pragma once
#include "RoadMath.h"
#include <vector>
class ArcRoad
{
public:
	ArcRoad(std::vector<VPE::dvec2>& inputPoints);
	~ArcRoad();

	//����arcRoad��
	std::vector<VPE::dvec2>& generatePoints();

	//�����������
	std::vector<VPE::dvec2> & getOutPoints() { return out_points; }
private:
	//��ʼ��һЩ����õ�����
	void Init(std::vector<double>& L, std::vector<double>&f);

	//�㷨����
	void alpha_assign(std::vector<double>& A, int s, int e, std::vector<double>& L, std::vector<double>& f);

	//����Aֵ����Բ��
	void generateArcPoints(int i, double dis);

	//����Ķ�άƽ���
	std::vector<VPE::dvec2> points;

	//����Ķ�άƽ���
	std::vector<VPE::dvec2> out_points;
};

