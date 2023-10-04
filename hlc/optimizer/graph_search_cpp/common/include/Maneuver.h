#pragma once

#include <vector>
#include <ColMajorMatrixAccessor.h>

namespace GraphBasedPlanning {
	template <typename T>
	struct Maneuver {
		std::vector<T> xs;
		std::vector<T> ys;
		std::vector<T> yaws;
		T dx;
		T dy;
		T dyaw;
		ColMajorMatrixAccessor<T> area;
		ColMajorMatrixAccessor<T> area_without_offset;
		ColMajorMatrixAccessor<T> area_large_offset;
	};
}  // namespace GraphBasedPlanning