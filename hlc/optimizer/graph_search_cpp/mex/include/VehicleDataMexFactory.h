#pragma once
#include <ConfigData.h>
#include <VehicleData.h>

#include <MatlabDataArray.hpp>
#include <mex.hpp>
#include <utility>

namespace GraphBasedPlanning {
	template <unsigned int n_vehicles, SCENARIO_TYPE scenario_type>
	std::array<VehicleData<scenario_type>, n_vehicles> make_vehicle_data(matlab::data::ObjectArray const& iter, std::shared_ptr<ConfigData const> const& config, std::shared_ptr<matlab::engine::MATLABEngine>& _matlab) {
		matlab::data::TypedArray<double> const reference_trajectory_points_array = _matlab->getProperty(iter, u"reference_trajectory_points");
		matlab::data::TypedArray<double> const v_ref_array = _matlab->getProperty(iter, u"v_ref");

		//std::vector<VehicleData<scenario_type>> vehicle_data;
		//vehicle_data.reserve(config->n_vehicles());
//
		//for (auto i = 0; i < config->n_vehicles(); ++i) {
		//	std::vector<vec2> trajectory_points(config->n_hp());
		//	for (auto j = 0; j < config->n_hp(); ++j) {
		//		trajectory_points[j].x = reference_trajectory_points_array[i][j][0];
		//		trajectory_points[j].y = reference_trajectory_points_array[i][j][1];
		//	}
//
		//	std::vector<double> v_ref(config->n_hp());
		//	for (auto j = 0; j < config->n_hp(); ++j) {
		//		v_ref[j] = v_ref_array[i][j];
		//	}
//
		//	if constexpr (scenario_type == SCENARIO_TYPE::Circle) {
		//		vehicle_data.emplace_back(std::move(trajectory_points), std::move(v_ref));
		//	} else {
		//		matlab::data::CellArray const predicted_lanelet_boundary_cell_array = _matlab->getProperty(iter, u"predicted_lanelet_boundary");
		//		matlab::data::TypedArray<double> const left_lanelet_array = std::move(predicted_lanelet_boundary_cell_array[i][0]);
		//		matlab::data::TypedArray<double> const right_lanelet_array = std::move(predicted_lanelet_boundary_cell_array[i][1]);
//
		//		vehicle_data.emplace_back(std::move(trajectory_points), std::move(v_ref),
		//		    std::move(ColMajorMatrixAccessor<double>(std::vector<double>(left_lanelet_array.begin(), left_lanelet_array.end()), left_lanelet_array.getDimensions())),
		//		    std::move(ColMajorMatrixAccessor<double>(std::vector<double>(right_lanelet_array.begin(), right_lanelet_array.end()), right_lanelet_array.getDimensions())));
		//	}
		//}
//
		//return vehicle_data;

		 auto produce_reference_trajectory_point = [&config, &reference_trajectory_points_array](std::size_t i) {
			std::vector<vec2> trajectory_points(config->n_hp());

			for (auto j = 0; j < config->n_hp(); ++j) {
				trajectory_points[j].x = reference_trajectory_points_array[i][j][0];
				trajectory_points[j].y = reference_trajectory_points_array[i][j][1];
			}

			return trajectory_points;
		};

		 auto produce_v_ref = [&config, &v_ref_array](std::size_t i) {
			std::vector<double> v_ref(config->n_hp());

			for (auto j = 0; j < config->n_hp(); ++j) {
				v_ref[j] = v_ref_array[i][j];
			}

			return v_ref;
		};

		 if constexpr (scenario_type == SCENARIO_TYPE::Circle) {
			auto produce_vehicle_data = [&produce_reference_trajectory_point, &produce_v_ref]<std::size_t... I>(
			                                std::index_sequence<I...>) -> std::array<VehicleData<scenario_type>, n_vehicles> { return {VehicleData<scenario_type>(produce_reference_trajectory_point(I), produce_v_ref(I))...}; };

			return produce_vehicle_data(std::make_index_sequence<n_vehicles>{});
		} else {
			matlab::data::CellArray const predicted_lanelet_boundary_cell_array = _matlab->getProperty(iter, u"predicted_lanelet_boundary");

			auto produce_predicted_left_lanelet_boundary = [&predicted_lanelet_boundary_cell_array](std::size_t i) {
				matlab::data::TypedArray<double> const left_lanelet_array = std::move(predicted_lanelet_boundary_cell_array[i][0]);

				return ColMajorMatrixAccessor<double>(std::vector<double>(left_lanelet_array.begin(), left_lanelet_array.end()), left_lanelet_array.getDimensions());
			};

			auto produce_predicted_right_lanelet_boundary = [&predicted_lanelet_boundary_cell_array](std::size_t i) {
				matlab::data::TypedArray<double> const right_lanelet_array = std::move(predicted_lanelet_boundary_cell_array[i][1]);

				return ColMajorMatrixAccessor<double>(std::vector<double>(right_lanelet_array.begin(), right_lanelet_array.end()), right_lanelet_array.getDimensions());
			};

			auto produce_vehicle_data = [&produce_reference_trajectory_point, &produce_v_ref, &produce_predicted_left_lanelet_boundary, &produce_predicted_right_lanelet_boundary]<std::size_t... I>(
			                                std::index_sequence<I...>) -> std::array<VehicleData<scenario_type>, n_vehicles> {
				return {VehicleData<scenario_type>(produce_reference_trajectory_point(I), produce_v_ref(I), produce_predicted_left_lanelet_boundary(I), produce_predicted_right_lanelet_boundary(I))...};
			};

			return produce_vehicle_data(std::make_index_sequence<n_vehicles>{});
		}
	}
}  // namespace GraphBasedPlanning
