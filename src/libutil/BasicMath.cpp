#include "BasicMath.h"

using namespace BM;

Point2 BM::operator+(const Point2 & pt1, const Point2 & pt2)
{
	Point2 out;
	out.x = pt1.x + pt2.x;
	out.y = pt1.y + pt2.y;
	return out;
}

Point2 BM::operator-(const Point2 & pt1, const Point2 & pt2)
{
	Point2 out;
	out.x = pt1.x - pt2.x;
	out.y = pt1.y - pt2.y;
	return out;
}

Point2 BM::operator*(double n, const Point2 & pt1)
{
	Point2 pt;
	pt.x = n*pt1.x;
	pt.y = n*pt1.y;
	return pt;
}

Point2 BM::operator/(const Point2 & pt, double n)
{
	Point2 out;

	out.x = pt.x / n;
	out.y = pt.y / n;
	return out;
}

std::ostream & BM::operator<<(std::ostream & output, const Point2 & pt)
{
	output << pt.x << ", " << pt.y << "\n";
	return output;
}

std::ostream & BM::operator<<(std::ostream & output, const Polar & pt)
{
	output << "(r: " << pt.r << ",theta: " << pt.theta << ")\n";
	return output;
}

std::ostream & BM::operator<<(std::ostream & output, const Point3 & pt)
{
	output << pt.x << ", " << pt.y <<", "<<pt.z<< "\n";
	return output;
}

Point3 BM::operator+(const Point3 & pt1, const Point3 & pt2)
{
	Point3 out;
	out.x = pt1.x + pt2.x;
	out.y = pt1.y + pt2.y;
	out.z = pt1.z + pt2.z;
	return out;
}

Point3 BM::operator-(const Point3 & pt1, const Point3 & pt2)
{
	Point3 out;
	out.x = pt1.x - pt2.x;
	out.y = pt1.y - pt2.y;
	out.z = pt1.z - pt2.z;
	return out;
}

Point3 BM::operator*(double n, const Point3 & pt1)
{
	Point3 out;
	out.x = n*pt1.x;
	out.y = n*pt1.y;
	out.z = n*pt1.z;

	return out;

}

Point3 BM::operator/(const Point3 & pt, double n)
{
	Point3 out;
	out.x = pt.x / n;
	out.y = pt.y / n;
	out.z = pt.z / n;
	return out;
}

double BM::Point2::dot(const Point2 & pt)
{
	return x*pt.x+y*pt.y;
}

double BM::Point2::mag()
{
	return std::sqrt(x*x+ y*y);
}
