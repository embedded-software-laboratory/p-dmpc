#pragma once
#include <ostream>

namespace GraphBasedPlanning {
	enum class SCENARIO_TYPE {
		Error,
		CommonRoad,
		Lanelet2,
		Circle,
	};

	std::ostream& operator<<(std::ostream& stream, SCENARIO_TYPE scenario_type);
}  // namespace GraphBasedPlanning