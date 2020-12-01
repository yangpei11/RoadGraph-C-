#pragma once
#include "RoadMath.h"
#include <vector>
class ArcRoad
{
public:
	ArcRoad(std::vector<VPE::dvec2>& inputPoints);
	~ArcRoad();

	//生成arcRoad点
	std::vector<VPE::dvec2>& generatePoints();

	//返回输出数据
	std::vector<VPE::dvec2> & getOutPoints() { return out_points; }
private:
	//初始化一些计算好的数据
	void Init(std::vector<double>& L, std::vector<double>&f);

	//算法主体
	void alpha_assign(std::vector<double>& A, int s, int e, std::vector<double>& L, std::vector<double>& f);

	//根据A值生成圆弧
	void generateArcPoints(int i, double dis);

	//输入的二维平面点
	std::vector<VPE::dvec2> points;

	//输出的二维平面点
	std::vector<VPE::dvec2> out_points;
};

