#pragma once
#define _USE_MATH_DEFINES

#include <cmath>
#include <iostream>
#include <algorithm>

namespace BM
{
	// ====== Point2 ========
	struct Point2
	{
		Point2() :x(0.0), y(0.0) {}
		double dot(const Point2 &pt);
		double mag();
		double x;
		double y;
	};
	Point2 operator +(const Point2 &pt1, const Point2 &pt2);
	Point2 operator -(const Point2 &pt1, const Point2 &pt2);
	Point2 operator *(double n, const Point2 &pt1);
	Point2 operator /(const Point2 &pt, double n);
	std::ostream &operator<<(std::ostream &output, const Point2 &pt);

	// ===== Polar ======
	struct Polar
	{
		Polar() : r(0), theta(0) {}
		double r;
		double theta;
	};
	std::ostream &operator<<(std::ostream &output, const Polar &pt);

	// ===== Point 3 =====
	struct Point3
	{
		Point3()
			:x(0), y(0), z(0) {}
		Point3(double x, double y, double z)
			: x(x), y(y), z(z) {}
		double x;
		double y;
		double z;
	};
	std::ostream &operator<<(std::ostream &output, const Point3 &pt);
	Point3 operator +(const Point3 &pt1, const Point3 &pt2);
	Point3 operator -(const Point3 &pt1, const Point3 &pt2);
	Point3 operator *(double n, const Point3 &pt1);
	Point3 operator /(const Point3 &pt, double n);
}
