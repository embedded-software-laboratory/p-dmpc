#pragma once

#include <ColMajorMatrixAccessor.h>
#include <vec2.h>

#include <vector>

#include "ScenarioType.h"

namespace GraphBasedPlanning {
	template <SCENARIO_TYPE scenario_type>
	struct predicted_lanelet_boundary_member {
		ColMajorMatrixAccessor<double> _predicted_left_lanelet_boundary;
		ColMajorMatrixAccessor<double> _predicted_right_lanelet_boundary;

		predicted_lanelet_boundary_member(decltype(_predicted_left_lanelet_boundary) &&predicted_left_lanelet_boundary, decltype(_predicted_right_lanelet_boundary) &&predicted_right_lanelet_boundary)
		    : _predicted_left_lanelet_boundary(predicted_left_lanelet_boundary), _predicted_right_lanelet_boundary(predicted_right_lanelet_boundary) {}
	};
	template <>
	struct predicted_lanelet_boundary_member<SCENARIO_TYPE::Circle> {};

	template <SCENARIO_TYPE scenario_type>
	class VehicleData : predicted_lanelet_boundary_member<scenario_type> {
		std::vector<vec2> _reference_trajectory_points;
		std::vector<double> _v_ref;

	   public:
		VehicleData(decltype(_reference_trajectory_points) &&reference_trajectory_points, decltype(_v_ref) &&v_ref) requires(scenario_type == SCENARIO_TYPE::Circle)
		    : _reference_trajectory_points(std::move(reference_trajectory_points)), _v_ref(std::move(v_ref)) {}
		VehicleData(decltype(_reference_trajectory_points) &&reference_trajectory_points, decltype(_v_ref) &&v_ref, ColMajorMatrixAccessor<double> &&predicted_left_lanelet_boundary,
		    ColMajorMatrixAccessor<double> &&predicted_right_lanelet_boundary) requires(scenario_type != SCENARIO_TYPE::Circle)
		    : predicted_lanelet_boundary_member<scenario_type>(
		          std::forward<decltype(predicted_left_lanelet_boundary)>(predicted_left_lanelet_boundary), std::forward<decltype(predicted_right_lanelet_boundary)>(predicted_right_lanelet_boundary)),
		      _reference_trajectory_points(std::move(reference_trajectory_points)),
		      _v_ref(std::move(v_ref)) {}

		[[nodiscard]] vec2 reference_trajectory_point(std::integral auto const step) const { return _reference_trajectory_points[step]; }

		[[nodiscard]] double v_ref(std::integral auto const step) const { return _v_ref[step]; }

		[[nodiscard]] ColMajorMatrixAccessor<double> const &predicted_left_lanelet_boundary() const requires(scenario_type != SCENARIO_TYPE::Circle) {
			return predicted_lanelet_boundary_member<scenario_type>::_predicted_left_lanelet_boundary;
		}
		[[nodiscard]] ColMajorMatrixAccessor<double> const &predicted_right_lanelet_boundary() const requires(scenario_type != SCENARIO_TYPE::Circle) {
			return predicted_lanelet_boundary_member<scenario_type>::_predicted_right_lanelet_boundary;
		}
	};
}  // namespace GraphBasedPlanning