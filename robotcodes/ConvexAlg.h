#pragma once

#include "BasicMath.h"
#include <vector>
#include <list>
using namespace BM;

namespace CH
{
	bool colinear3PtTest(const Point2 &pt1, const Point2 &pt2, const Point2 &pt3);
	std::vector<Polar> getConvexPolars(const std::vector<Polar> &polarList);
	std::vector<Point2> grahamConvexHull(std::vector<Point2> points);
}