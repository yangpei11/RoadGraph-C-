#pragma once
#include <vector>
#include "glm/glm.hpp"

class Vehicle;

// \brief: 表达一条车道的结构
struct Lane
{
	Lane* left = nullptr;		//左侧车道
	Lane* right = nullptr;	//右侧车道
	Lane* previous = nullptr;	//前序车道
	Lane* next = nullptr;		//后序车道
	std::vector<glm::dvec3> points_;		//车道线上的全局坐标点

	std::vector<Vehicle*> vehicles_;	//当前该车道上的车辆
};