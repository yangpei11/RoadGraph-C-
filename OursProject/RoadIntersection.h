#pragma once
#include <algorithm>
#include <glm/gtc/constants.hpp>
#include "glm/gtc/type_ptr.hpp"
#include <glm/glm.hpp>
#include "RoadNode.h"

class RoadIntersection
{
public:
	RoadIntersection(double D, RoadNode* point) {
		d = D;
		point_ = point;
	}
	void generatePoints();
	double d;
	RoadNode* point_;
	std::vector<double> thetas;
	std::vector< std::vector<VPE::dvec2> > boundaries;
	std::vector<double> L_array;
};

class Node {
public:
	Node(double x, double y, double nx, double ny, double theta, RoadNode* p)
	{
		this->x = x;
		this->y = y;
		this->nx = nx;
		this->ny = ny;
		this->theta = theta;
		this->p = p;
	}
	double x, y, nx, ny, theta;
	RoadNode* p;
};
