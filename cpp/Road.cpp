// Road.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include "ArcRoad.h"
#include "CublicSpline.h"

int main()
{
	std::vector<VPE::dvec2> g;
	g.emplace_back(0, 0);
	g.emplace_back(1, 1);
	g.emplace_back(3, 1);
	g.emplace_back(4, 2);
	ArcRoad road(g);
	road.generatePoints();
	auto points2D = road.getOutPoints();
	auto points3D = CublicSpline::ConstuctArchBridge(points2D, 0, 0, 10, 0.1);
	return 0;
}
