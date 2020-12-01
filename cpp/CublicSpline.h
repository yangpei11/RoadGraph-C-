#pragma once
#include "RoadMath.h"
#include <vector>

//利用三次样条技术构造
namespace CublicSpline
{
	/* 构造普通的上升或者下降的桥梁
	 * @param  points2d 二维化的点
	 * @param  h1，h2为两端的高度
	 * @param  step 迭代需求，根据自己算法选择合适的步长。
	 * @return 三维化的点
	 */
	static std::vector<VPE::dvec3>& ConstuctOrdinaryBridge(std::vector<VPE::dvec2> &points2d, double h1, double h2, double step)
	{
		std::vector<VPE::dvec3> outPoints3d;
		double length = 0;
		for (auto i = 1; i < points2d.size(); ++i) {
			length += VPE::distance(points2d[i - 1], points2d[i]);
		}

		//三次样条
		double x1 = 0, y1 = h1, x2 = length, y2 = h2;
		double a = y1, b = 0, c = 3 * (y2 - y1) / ((x2 - x1) * (x2 - x1));
		double d = -2 * (y2 - y1) / ((x2 - x1) * (x2 - x1) * (x2 - x1));

		double addLength = 0;
		//迭代步长
		double addStep = step;
		for (int i = 1; i < points2d.size(); ++i) {
			double gapLength = VPE::distance(points2d[i - 1], points2d[i]);
			double futureLength = addLength + gapLength;
			auto  n = VPE::normalize(points2d[i] - points2d[i - 1]);

			for (double dis = 0; dis < gapLength; dis += addStep) {

				VPE::dvec2 localPos = points2d[i - 1] +  n*(double)dis;

				double insertHeight = a + c * (addLength - x1) * (addLength - x1) +
					d * (addLength - x1) * (addLength - x1) * (addLength - x1);
				outPoints3d.emplace_back(localPos.x, localPos.y, insertHeight);
				addLength += addStep;
			}
			addLength = futureLength;
		}

		outPoints3d.emplace_back(points2d.back().x, points2d.back().y, h2);

		return outPoints3d;
	}

	/* 构造拱桥
	 * @param  points2d 二维化的点
	 * @param  h1，h2为两端的高度， midPointHeight为拱桥最高点高度
	 * @param  step 迭代需求，根据自己算法选择合适的步长。
	 * @return 三维化的点
	 */
	static std::vector<VPE::dvec3> ConstuctArchBridge(std::vector<VPE::dvec2> &points2d, double h1, double h2, double midPointHeight, double step)
	{
		std::vector<VPE::dvec3> outPoints3d;
		double length = 0;
		for (auto i = 1; i < points2d.size(); ++i)
		{
			length += VPE::distance(points2d[i - 1], points2d[i]);
		}
		double half_len = length / 2.0;


		// 三次样条方程
	    //当addLength < length/2.0时
		double x1 = 0, y1 = h1, x2 = half_len, y2 = midPointHeight;
		double a = y1, b = 0, c = 3 * (y2 - y1) / ((x2 - x1) * (x2 - x1));
		double d = -2 * (y2 - y1) / ((x2 - x1) * (x2 - x1) * (x2 - x1));

		//当addLength > length/2.0时
		double m1 = 0, n1 = midPointHeight, m2 = length / 2.0, n2 = h2;
		double a1 = n1, b1 = 0, c1 = 3 * (n2 - n1) / ((m2 - m1) * (m2 - m1));
		double d1 = -2 * (n2 - n1) / ((m2 - m1) * (m2 - m1) * (m2 - m1));

		bool is_first_half = true; // 是否前半段
		double addLength = 0;
		//迭代步长
		double addStep = step;
		for (int i = 1; i < points2d.size(); ++i) {
			double gapLength = VPE::distance(points2d[i - 1], points2d[i]);
			double futureLength = addLength + gapLength;
			auto   n = VPE::normalize(points2d[i] - points2d[i - 1]);
			for (double dis= 0; dis < gapLength; dis += addStep) {
				if (addLength > half_len && is_first_half) {
					is_first_half = false; //改为后半段
				}
				VPE::dvec2 localPos = points2d[i - 1] + n* double(dis);

				double insertHeight;
				if (is_first_half) {
					insertHeight = a + c * (addLength - x1) * (addLength - x1) +
						d * (addLength - x1) * (addLength - x1) * (addLength - x1);
				}
				else {
					insertHeight = a1 +
						c1 * (addLength - half_len - m1) * (addLength - half_len - m1) +
						d1 * (addLength - half_len - m1) * (addLength - half_len - m1) *
						(addLength - half_len - m1);
				}
				outPoints3d.emplace_back(localPos.x, localPos.y, insertHeight);
				addLength += addStep;
			}
			addLength = futureLength;
		}

		outPoints3d.emplace_back(points2d.back().x, points2d.back().y, h2);

		return outPoints3d;
	}
}
	