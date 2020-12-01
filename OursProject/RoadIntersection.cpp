#include "RoadIntersection.h"
#include <vector>

bool cmp(Node& a, Node& b) {
	return a.theta < b.theta;
}

void RoadIntersection::generatePoints() {
	std::vector <Node> road;
	for (auto nextPoint : point_->connectNodes) {
		VPE::dvec2 v = VPE::dvec2(nextPoint->localCoord.x - point_->localCoord.x, nextPoint->localCoord.z - point_->localCoord.z);
		VPE::dvec2 l = VPE::normalize(v);
		double theta;
		if (v.x == 0) {
			if (v.y >= 0) {
				theta = 0.5 * VPE::PI;
			}
			else {
				theta = 1.5 * VPE::PI;
			}
		}
		else {
			theta = VPE::atan(v.y / v.x);
		}
		if (v.x < 0) {
			theta = VPE::PI + theta;
		}
		road.emplace_back(nextPoint->localCoord.x, nextPoint->localCoord.z, l.x, l.y, theta, nextPoint);
	}

	std::sort(road.begin(), road.end(), cmp);
	double thres = 8.0;
	for (int i = 0; i < road.size(); i++) {
		double ti;
		int idx;
		if (i == 0) {
			idx = road.size() - 1;
		}
		else {
			idx = i - 1;
		}
		ti = road[i].theta - road[idx].theta;
		if (ti < 0) {
			ti += 2 * VPE::PI;
		}
		thetas.push_back(ti);
		double L1 = VPE::distance(VPE::dvec2(road[idx].x, road[idx].y), VPE::dvec2(point_->localCoord.x, point_->localCoord.z));
		double L2 = VPE::distance(VPE::dvec2(road[i].x, road[i].y), VPE::dvec2(point_->localCoord.x, point_->localCoord.z));
		if (fabs(ti - VPE::PI) < 0.08) {
			L_array.push_back(0);
			continue;
		}
		auto Lc = (4.5 + d) / VPE::tan(ti / 2.0);
		if (VPE::min(L1, L2) > Lc && Lc < thres) {
			L_array.push_back(VPE::min(VPE::min(L1, L2), thres));
			continue;
		}
		auto result = VPE::min(VPE::min(L1, L2), Lc);
		L_array.push_back(result);
	}

	for (int i = 0; i < road.size(); i++) {
		double L = L_array[i];
		VPE::dvec2 O(point_->localCoord.x, point_->localCoord.z);
		if (fabs(VPE::PI - thetas[i]) > 0.08) {
			std::vector<VPE::dvec2> points;
			VPE::dvec3 n1, n2;
			if (i == 0) {
				n1 = VPE::dvec3(road[road.size() - 1].nx, road[road.size() - 1].ny, 0);
			}
			else {
				n1 = VPE::dvec3(road[i - 1].nx, road[i - 1].ny, 0);
			}
			n2 = VPE::dvec3(road[i].nx, road[i].ny, 0);
			auto o = VPE::normalize(VPE::cross(n1, n2));
			auto lam = VPE::sign(o.z);
			auto p = VPE::cross(n1, o) * (-lam);
			auto p2 = VPE::cross(n2, o) * (-lam);
			auto idx = i == 0 ? (road.size() - 1) : (i - 1);
			//²âÊÔ´úÂë£¬ÒªÉ¾³ý
			points.push_back(VPE::dvec2(road[idx].x + d * p.x, road[idx].y + d * p.y));

			VPE::dvec3 n3 = VPE::normalize(n1 + n2);
			VPE::dvec2 center;
			if (lam >= 0) {
				center = VPE::dvec2(O.x + n3.x * L / VPE::cos(0.5 * thetas[i]), O.y + n3.y * L / VPE::cos(0.5 * thetas[i]));
			}
			else {
				center = VPE::dvec2(O.x + n3.x * L / VPE::cos(VPE::PI - 0.5 * thetas[i]), O.y + n3.y * L / VPE::cos(VPE::PI - 0.5 * thetas[i]));
			}

			if (i == 0) {
				points.push_back(VPE::dvec2(O.x + L * road[road.size() - 1].nx + d * p.x, O.y + L * road[road.size() - 1].ny + d * p.y));
			}
			else {
				points.push_back(VPE::dvec2(O.x + L * road[i - 1].nx + d * p.x, O.y + L * road[i - 1].ny + d * p.y));
			}

			for (int j = 1; j < int(lam * (VPE::PI - thetas[i]) / 0.1); j++) {
				double x0 = (points[1].x - center.x) * VPE::cos(-lam * j * 0.1) - (points[1].y - center.y) * VPE::sin(-lam * j * 0.1) + center.x;
				double y0 = (points[1].x - center.x) * VPE::sin(-lam * j * 0.1) + (points[1].y - center.y) * VPE::cos(-lam * j * 0.1) + center.y;
				points.push_back(VPE::vec2(x0, y0));
			}
			points.push_back(VPE::dvec2(O.x + L * road[i].nx - d * p2.x, O.y + L * road[i].ny - d * p2.y));
			points.push_back(VPE::dvec2(road[i].x - d * p2.x, road[i].y - d * p2.y));
			boundaries.push_back(points);
		}
		else {
			int index1, index2;
			if (i == 0) {
				index1 = road.size() - 2;
				index2 = road.size() - 1;
			}
			else if (i == 1) {
				index1 = road.size() - 1;
				index2 = 0;
			}
			else {
				index1 = i - 2;
				index2 = i - 1;
			}

			VPE::dvec3 n1 = VPE::dvec3(road[index1].nx, road[index1].ny, 0);
			VPE::dvec3 n2 = VPE::dvec3(road[index2].nx, road[index2].ny, 0);
			auto o = VPE::normalize(VPE::cross(n2, n1));
			auto p = VPE::cross(n1, o);
			auto p2 = VPE::cross(n2, o);
			std::vector<VPE::dvec2> points;
			//²âÊÔ´úÂë£¬ÒªÉ¾³ý
			points.emplace_back(road[index2].x + d * p2.x, road[index2].y + d * p2.y);

			points.push_back(VPE::dvec2(O.x + L * road[index2].nx + d * p2.x, O.y + L * road[index2].ny + d * p2.y));
			points.push_back(VPE::dvec2(O.x + L * road[i].nx + d * p2.x, O.y + L * road[i].ny + d * p2.y));

			//²âÊÔ´úÂë£¬ÒªÉ¾³ý
			points.emplace_back(road[i].x + d * p2.x, road[i].y + d * p2.y);
			boundaries.push_back(points);
		}
	}
}