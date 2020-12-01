#pragma once
#include "RoadMath.h"
#include <vector>

//��������������������
namespace CublicSpline
{
	/* ������ͨ�����������½�������
	 * @param  points2d ��ά���ĵ�
	 * @param  h1��h2Ϊ���˵ĸ߶�
	 * @param  step �������󣬸����Լ��㷨ѡ����ʵĲ�����
	 * @return ��ά���ĵ�
	 */
	static std::vector<VPE::dvec3>& ConstuctOrdinaryBridge(std::vector<VPE::dvec2> &points2d, double h1, double h2, double step)
	{
		std::vector<VPE::dvec3> outPoints3d;
		double length = 0;
		for (auto i = 1; i < points2d.size(); ++i) {
			length += VPE::distance(points2d[i - 1], points2d[i]);
		}

		//��������
		double x1 = 0, y1 = h1, x2 = length, y2 = h2;
		double a = y1, b = 0, c = 3 * (y2 - y1) / ((x2 - x1) * (x2 - x1));
		double d = -2 * (y2 - y1) / ((x2 - x1) * (x2 - x1) * (x2 - x1));

		double addLength = 0;
		//��������
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

	/* ���칰��
	 * @param  points2d ��ά���ĵ�
	 * @param  h1��h2Ϊ���˵ĸ߶ȣ� midPointHeightΪ������ߵ�߶�
	 * @param  step �������󣬸����Լ��㷨ѡ����ʵĲ�����
	 * @return ��ά���ĵ�
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


		// ������������
	    //��addLength < length/2.0ʱ
		double x1 = 0, y1 = h1, x2 = half_len, y2 = midPointHeight;
		double a = y1, b = 0, c = 3 * (y2 - y1) / ((x2 - x1) * (x2 - x1));
		double d = -2 * (y2 - y1) / ((x2 - x1) * (x2 - x1) * (x2 - x1));

		//��addLength > length/2.0ʱ
		double m1 = 0, n1 = midPointHeight, m2 = length / 2.0, n2 = h2;
		double a1 = n1, b1 = 0, c1 = 3 * (n2 - n1) / ((m2 - m1) * (m2 - m1));
		double d1 = -2 * (n2 - n1) / ((m2 - m1) * (m2 - m1) * (m2 - m1));

		bool is_first_half = true; // �Ƿ�ǰ���
		double addLength = 0;
		//��������
		double addStep = step;
		for (int i = 1; i < points2d.size(); ++i) {
			double gapLength = VPE::distance(points2d[i - 1], points2d[i]);
			double futureLength = addLength + gapLength;
			auto   n = VPE::normalize(points2d[i] - points2d[i - 1]);
			for (double dis= 0; dis < gapLength; dis += addStep) {
				if (addLength > half_len && is_first_half) {
					is_first_half = false; //��Ϊ����
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
	