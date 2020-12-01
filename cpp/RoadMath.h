#pragma once
#include <cmath>

namespace VPE
{
	inline double sin(double const& angle)
	{
		return std::sin(angle);
	}

	inline double cos(double const& angle)
	{
		return std::cos(angle);
	}

	inline double radians(double const& degrees)
	{
		const double pi = 3.141592653589;
		return degrees * (pi / 180.0);
	}

	class dvec2
	{
	public:
		dvec2(double x, double y):x(x),y(y){}
		dvec2 operator + (const dvec2& b)
		{
			return dvec2(x + b.x, y + b.y);
		}
		dvec2 operator - (const dvec2& b)
		{
			return dvec2(x - b.x, y - b.y);
		}
		dvec2 operator * (double n)
		{
			return dvec2(x*n, y*n);
		}
	public:
		double x, y;
	};

	class dvec3
	{
	public:
		dvec3(double x, double y, double z) :x(x), y(y), z(z) {}
		dvec3 operator + (const dvec3& b)
		{
			return dvec3(x + b.x, y + b.y, z+b.z);
		}
		dvec3 operator - (const dvec3& b)
		{
			return dvec3(x - b.x, y - b.y, z-b.z);
		}
	public:
		double x, y,z;
	};

	inline double length(dvec2& vec)
	{
		return std::sqrt(vec.x*vec.x+vec.y*vec.y);
	}
	
	inline double length(dvec2&& vec)
	{
		return std::sqrt(vec.x*vec.x + vec.y*vec.y);
	}

	inline dvec2 normalize(dvec2& vec)
	{
		double l = length(vec);
		return dvec2(vec.x/l, vec.y/l);
	}

	inline dvec2 normalize(dvec2&& vec)
	{
		double l = length(vec);
		return dvec2(vec.x / l, vec.y / l);
	}

	inline double dot(dvec2 &a, dvec2& b)
	{
		return a.x*b.x + a.y*b.y;
	}

	inline double max(double a, double b)
	{
		return a < b ? b : a;
	}

	inline double min(double a, double b)
	{
		return a > b ? b : a;
	}

	inline double asin(double radians)
	{
		return std::asin(radians);
	}

	inline double acos(double radians)
	{
		return std::acos(radians);
	}

	inline double sqrt(double a)
	{
		return std::sqrt(a);
	}

	inline double distance(VPE::dvec2& a, VPE::dvec2& b)
	{
		return VPE::length(a - b);
	}
}
