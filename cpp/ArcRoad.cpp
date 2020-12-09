#pragma once
#include "pch.h"
#include "ArcRoad.h"


//过滤三点一线的中间点
ArcRoad::ArcRoad(std::vector<VPE::dvec2>& inputPoints)
{
	double cos_thres = VPE::cos(VPE::radians(179.0f));
	if(inputPoints.size() != 2)
	{
		points.emplace_back(inputPoints.front().x, inputPoints.front().y);

		int pre = 0;
		for (int i = 1; i < inputPoints.size()-1; i++)
		{
			VPE::dvec2 l1(inputPoints[pre].x - inputPoints[i].x, inputPoints[pre].y - inputPoints[i].y);
			VPE::dvec2 l2(inputPoints[i + 1].x - inputPoints[i].x, inputPoints[i + 1].y - inputPoints[i].y);
			if (VPE::length(l1) < 0.01 || VPE::length(l2) < 0.01)
			{
				continue;
			}
			l1 = VPE::normalize(l1);
			l2 = VPE::normalize(l2);
			//共线的话去掉该点
			if (VPE::dot(l1, l2) > cos_thres) {
				this->points.emplace_back(inputPoints[i].x, inputPoints[i].y);
				pre = i;
			}
		}


		points.emplace_back(inputPoints.back().x, inputPoints.back().y);
	}
	else
	{
		points.emplace_back(inputPoints.front().x, inputPoints.front().y);
		points.emplace_back(inputPoints.back().x, inputPoints.back().y);
	}
}

void ArcRoad::Init(std::vector<double>& L, std::vector<double>&f)
{
	for (int i = 1; i < points.size() - 1; i++)
	{
		if (i == 1)
		{
			f.push_back(0);
		}
		VPE::dvec2 v1 = points[i] - points[i - 1];
		VPE::dvec2 v2 = points[i + 1] - points[i];
		VPE::dvec2 n1 = VPE::normalize(v1);
		VPE::dvec2 n2 = VPE::normalize(v2);
		L.push_back(VPE::length(v1));
		if (i == points.size() - 2) {
			L.push_back(VPE::length(v2));
		}
		f.push_back(std::sqrt((1 + VPE::dot(n1, n2)) / (1 - VPE::dot(n1, n2))));
		if (i == points.size() - 2) {
			f.push_back(0);
		}
	}
}

void ArcRoad::alpha_assign(std::vector<double>& A, int s, int e, std::vector<double>& L, std::vector<double>& f) {
	double r_min = 1 << 30;
	int i_min = e;
	double alpha_low = 0;
	double alpha_high = 0;
	if (s + 1 >= e) {
		return;
	}
	double alpha_b = VPE::min(L[s] - A[s], L[s + 1]);
	double r_current = VPE::max(f[s] * A[s], f[s + 1] * alpha_b);
	if (r_current < r_min) {
		r_min = r_current;
		i_min = s;
		alpha_low = A[s];
		alpha_high = alpha_b;
	}

	for (int i = s + 1; i <= e - 2; i++) {
		double alpha_a = VPE::min(L[i - 1], L[i] * f[i] / (f[i] + f[i + 1]));
		double alpha_b = VPE::min(L[i + 1], L[i] - alpha_a);
		double r_current = VPE::max(f[i] * alpha_a, f[i + 1] * alpha_b);
		if (r_current < r_min) {
			r_min = r_current;
			i_min = i;
			alpha_low = alpha_a;
			alpha_high = alpha_b;
		}
	}

	double alpha_a = VPE::min(L[e - 2], L[e - 1] - A[e]);
	r_current = VPE::max(f[e - 1] * alpha_a, f[e] * A[e]);
	if (r_current < r_min) {
		r_min = r_current;
		i_min = e - 1;
		alpha_low = alpha_a;
		alpha_high = A[e];
	}
	A[i_min] = alpha_low;
	A[i_min + 1] = alpha_high;
	alpha_assign(A, s, i_min, L, f);
	alpha_assign(A, i_min + 1, e, L, f);
}

void ArcRoad::generateArcPoints(int i, double dis) {
	VPE::dvec2 n1 = VPE::normalize(VPE::dvec2(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y));
	VPE::dvec2 n2 = VPE::normalize(VPE::dvec2(points[i + 1].x - points[i].x, points[i + 1].y - points[i].y));
	VPE::dvec2 b = VPE::normalize(n2 - n1);
	double beishu = dis * VPE::sqrt((1 + VPE::dot(n1, n2)) / (1 - VPE::dot(n1, n2)) + 1);
	VPE::dvec2 O = VPE::dvec2(points[i].x, points[i].y) + VPE::dvec2(b.x * beishu, b.y * beishu);
	VPE::dvec2 M = VPE::dvec2(points[i].x, points[i].y) + VPE::dvec2(n1.x * (-dis), n1.y * (-dis));
	VPE::dvec2 N = VPE::dvec2(points[i].x, points[i].y) + VPE::dvec2(n2.x * dis, n2.y * dis);
	VPE::dvec2 OM = M - O;
	VPE::dvec2 ON = N - O;
	VPE::dvec2 start = OM;
	double angle = VPE::acos(VPE::dot(OM, ON) / (VPE::length(OM) * VPE::length(ON)));
	double newX = OM.x * VPE::cos(angle) - OM.y * VPE::sin(angle);
	double newY = OM.x * VPE::sin(angle) + OM.y * VPE::cos(angle);
	bool roateFlag;
	if (fabs(newX - ON.x) < 0.01 && fabs(newY - ON.y) < 0.01) {
		roateFlag = true;
	}
	else {
		roateFlag = false;
	}
	
	VPE::dvec2 start1 = OM;
	/*VPE::dvec2 collinearPoint = O + start1;
	VPE::dvec2 v1 = collinearPoint - out_points.back();
	VPE::dvec2 v2 = VPE::dvec2(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y);
	*/

	double roateAngle = 0.0;
	double tmp;
	//double step = 5.0 / VPE::length(OM); 步长为5m
	//double step = 2 * VPE::asin(2.5 / VPE::length(OM));
	double step = 0.01;
	while (roateAngle <= angle) {
		tmp = roateAngle;
		if (roateFlag == 0) {
			roateAngle = -roateAngle;
		}
		double x1 = start1.x * VPE::cos(roateAngle) - start1.y * VPE::sin(roateAngle);
		double y1 = start1.x * VPE::sin(roateAngle) + start1.y * VPE::cos(roateAngle);

		out_points.push_back(VPE::dvec2(x1, y1) + O);

		roateAngle = tmp;
		roateAngle += step;
	}
}

std::vector<VPE::dvec2> &ArcRoad::generatePoints() {

	if (points.size() == 2) {
		if (VPE::distance(points[0], points[1]) < 0.01) {
			return out_points;
		}
		out_points.push_back(points[0]);
		out_points.push_back(points[1]);
	}
	else {
		int n = points.size();
		std::vector<double> A(n, 0);
		std::vector<double> L;
		std::vector<double> f;
		Init(L, f);
		alpha_assign(A, 0, n - 1, L, f);
		//加上开始点
		out_points.push_back(points[0]);

		for (int i = 1; i < n - 1; i++) {
			generateArcPoints(i, A[i]);
		}
		//加上最后一个点
		out_points.push_back(points.back());
	}
	return out_points;
}

ArcRoad::~ArcRoad()
{

}
