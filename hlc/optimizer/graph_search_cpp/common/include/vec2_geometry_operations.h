#pragma once

#include <vec2.h>
#include <vec2_boost_adapter.h>
#include <limits>
#include <vector>

#include "ScenarioType.h"

namespace GraphBasedPlanning {
	template <SCENARIO_TYPE scenario_type, int look_before, int look_after>
	inline std::tuple<vec2, int> closest_point(vec2 const point, std::vector<vec2> const& reference_trajectory_points, int const current_index_on_trajectory) {
		int const n_reference_trajectory_line_strips = static_cast<int>(reference_trajectory_points.size());

		double min_distance = std::numeric_limits<double>::infinity();
		int min_index = -1;
		int min_index_next = -1;
		if constexpr (scenario_type == SCENARIO_TYPE::Circle) {
			for (auto i = std::max<int>(current_index_on_trajectory - look_before, 0), j = i + 1; j <= std::min<int>(current_index_on_trajectory + look_after, n_reference_trajectory_line_strips - 1); ++i, ++j) {
				std::vector<vec2> test = {reference_trajectory_points[i], reference_trajectory_points[j]};
				auto const distance = boost::geometry::comparable_distance(point, test);
				if (distance < min_distance) {
					min_distance = distance;
					min_index = i;
					min_index_next = j;
				}
			}
		}

		if constexpr (scenario_type == SCENARIO_TYPE::CommonRoad) {  // roundtrip
			auto i_start = (((current_index_on_trajectory - look_before) % n_reference_trajectory_line_strips) + n_reference_trajectory_line_strips) % n_reference_trajectory_line_strips;
			for (auto i = i_start, j = (i_start + 1) % n_reference_trajectory_line_strips, k = 0; k <= look_before + look_after && (k == 0 || i != i_start); ++k, i = (i + 1) % n_reference_trajectory_line_strips, j = (j + 1) % n_reference_trajectory_line_strips) {
				std::vector<vec2> test = {reference_trajectory_points[i], reference_trajectory_points[j]};
				auto const distance = boost::geometry::comparable_distance(point, test);
				if (distance < min_distance) {
					min_distance = distance;
					min_index = i;
					min_index_next = j;
				}
			}
		}

		boost::geometry::model::segment<vec2> res;
		std::vector<vec2> test = {reference_trajectory_points[min_index], reference_trajectory_points[min_index_next]};
		boost::geometry::closest_points(point, test, res);

		return std::tie(res.second, min_index);
	}

	double trajectory_distance(vec2 from, unsigned int index_from, vec2 to, unsigned int index_to, std::vector<vec2> const& reference_trajectory_points);
}