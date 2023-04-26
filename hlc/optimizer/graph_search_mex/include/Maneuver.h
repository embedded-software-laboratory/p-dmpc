#pragma once

#include <vector>

#include "ColMajorAccessor.h"

namespace GraphBasedPlanning {
	struct Maneuver {
		std::vector<double> xs;
		std::vector<double> ys;
		std::vector<double> yaws;
		double dx;
		double dy;
		double dyaw;
		ColMajorMatrixAccessor<double> area;
		ColMajorMatrixAccessor<double> area_without_offset;
		ColMajorMatrixAccessor<double> area_large_offset;
	};
}  // namespace GraphBasedPlanning