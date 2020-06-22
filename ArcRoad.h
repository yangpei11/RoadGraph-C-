#pragma once
#include <vector>
#include "RoadNode.h"
#include "RoadEdge.h"
class ArcRoad
{	
public:
    ArcRoad(RoadEdge *e);
    std::vector<VPE::dvec2> & generatePoints();

private:
	void generateArcPoints(int i, double dis);
	void Init(std::vector<double>& L, std::vector<double>& f);
	void alpha_assign(std::vector<double>& A, int s, int e, std::vector<double>& L, std::vector<double>& f);
	std::vector<VPE::dvec2> points;
	std::vector<VPE::dvec2> out_points;
    RoadEdge *                             edge;
	
	std::vector < std::pair<double, int> > arcInformation;// 单位弧长和弧长个数
	std::vector< std::pair<int, int> > arcPos;
};

