#pragma once

#include <functional>
#include <utility>

#include "GraphSearch.h"
#include "MPA_MEX.h"
#include "MatlabDataArray.hpp"

namespace GraphBasedPlanning {
	class GraphSearchMEX : protected virtual GraphSearch, private MPA_MEX {
	   private:
		[[nodiscard]] inline virtual unsigned int &n_hp() = 0;
		[[nodiscard]] inline virtual unsigned int n_hp() const = 0;

		[[nodiscard]] inline virtual unsigned int &n_vehicles() = 0;
		[[nodiscard]] inline virtual unsigned int n_vehicles() const = 0;

		[[nodiscard]] inline virtual unsigned int &tick_per_step() = 0;
		[[nodiscard]] inline virtual unsigned int tick_per_step() const = 0;

		[[nodiscard]] inline virtual double &dt() = 0;
		[[nodiscard]] inline virtual double dt() const = 0;

		[[nodiscard]] inline virtual SCENARIO_TYPE &scenario_type() = 0;
		[[nodiscard]] inline virtual SCENARIO_TYPE scenario_type() const = 0;

		[[nodiscard]] inline virtual std::vector<std::vector<CollisionDetection::vec2>> &reference_trajectory() = 0;
		[[nodiscard]] inline virtual std::vector<std::vector<CollisionDetection::vec2>> const &reference_trajectory() const = 0;

		[[nodiscard]] inline virtual bool &is_pb() = 0;
		[[nodiscard]] inline virtual bool is_pb() const = 0;

		[[nodiscard]] inline virtual bool &recursive_feasibility() = 0;
		[[nodiscard]] inline virtual bool recursive_feasibility() const = 0;

		matlab::data::ArrayFactory &_factory;
		std::shared_ptr<matlab::engine::MATLABEngine> &_matlab;

	   protected:
		GraphSearchMEX(matlab::data::ArrayFactory &factory, std::shared_ptr<matlab::engine::MATLABEngine> &_matlab) : MPA_MEX(_matlab), _factory(factory), _matlab(_matlab) {}

		void scenario_callback(matlab::data::Array const &scenario, matlab::data::Array const &mpa_array) {
			mpa_callback(mpa_array);

			matlab::data::Array options_array = _matlab->getProperty(scenario, u"options");
			matlab::data::TypedArray<double> const tick_per_step_array = _matlab->getProperty(options_array, u"tick_per_step");
			tick_per_step() = static_cast<unsigned int>(tick_per_step_array[0][0]);
			matlab::data::TypedArray<double> const dt_array = _matlab->getProperty(options_array, u"dt_seconds");
			dt() = static_cast<double>(dt_array[0][0]);
			matlab::data::TypedArray<bool> is_pb_array = _matlab->getProperty(options_array, u"is_prioritized");
			is_pb() = static_cast<bool>(is_pb_array[0][0]);

			matlab::data::Array const vehicles_array = _matlab->getProperty(scenario, u"vehicles");
			if (is_pb()) {
				n_vehicles() = 1;
			} else {
				n_vehicles() = vehicles_array.getNumberOfElements();
			}
			Node<FLOATING_POINT_TYPE>::init_static_variables(n_vehicles());

			matlab::data::TypedArray<bool> recursive_feasibility_array = _matlab->getProperty(options_array, u"recursive_feasibility");
			recursive_feasibility() = static_cast<bool>(recursive_feasibility_array[0][0]);

			reference_trajectory().resize(n_vehicles());
			for (unsigned int i = 0; i < n_vehicles(); ++i) {
				matlab::data::TypedArray<double> const reference_trajectory_array = _matlab->getProperty(vehicles_array, i, u"reference_trajectory");
				reference_trajectory()[i].resize(reference_trajectory_array.getDimensions()[0]);

				// bl�d, dass das Format in Matlab schlecht gew�hlt ist, sodass das folgende n�tig wird:
				for (unsigned int j = 0; j < reference_trajectory()[i].size(); ++j) {
					reference_trajectory()[i][j].x = reference_trajectory_array[j][0];
					reference_trajectory()[i][j].y = reference_trajectory_array[j][1];
				}
			}

			matlab::data::EnumArray const scenario_type_array = _matlab->getProperty(options_array, u"scenario_type");
			std::string scenario_type_string = std::string(scenario_type_array[0]);

			if (scenario_type_string == "commonroad") {
				scenario_type() = SCENARIO_TYPE::CommonRoad;
			} else if (scenario_type_string == "circle") {
				scenario_type() = SCENARIO_TYPE::Circle;
			} else {
				throw MexException("Scenario type '", scenario_type_string, "' not implemented/available!");
			}
		}

		std::tuple<matlab::data::CellArray, matlab::data::TypedArray<double>, matlab::data::CellArray, matlab::data::TypedArray<uint32_t>, matlab::data::TypedArray<bool>> graph_search_callback_centralized(
		    matlab::data::ObjectArray const &iter, const std::function<Node<FLOATING_POINT_TYPE> const *(ColMajorMatrixAccessor<FLOATING_POINT_TYPE> const &, std::vector<std::uint8_t> const &)> &find,
		    const std::function<uint64_t(void)> &get_n_expanded) {
			matlab::data::TypedArray<double> trim_indices_array = std::move(iter[0][0]["trim_indices"]);
			std::vector<std::uint8_t> trim_indices(trim_indices_array.begin(), trim_indices_array.end());

			for (auto &e : trim_indices) {
				e -= 1;
			}

			matlab::data::TypedArray<double> x0_array = std::move(iter[0][0]["x0"]);
			ColMajorMatrixAccessor<double> x0 = ColMajorMatrixAccessor<double>(std::vector<double>(x0_array.begin(), x0_array.end()), x0_array.getDimensions());

			matlab::data::TypedArray<double> reference_trajectory_points_array = std::move(iter[0][0]["reference_trajectory_points"]);
			_reference_trajectory_points = ColMajorTensorAccessor<double>(std::vector<double>(reference_trajectory_points_array.begin(), reference_trajectory_points_array.end()), reference_trajectory_points_array.getDimensions());

			matlab::data::TypedArray<double> v_ref_array = std::move(iter[0][0]["v_ref"]);
			_v_ref = ColMajorMatrixAccessor<double>(std::vector<double>(v_ref_array.begin(), v_ref_array.end()), v_ref_array.getDimensions());

			if (scenario_type() == SCENARIO_TYPE::CommonRoad) {
				matlab::data::CellArray predicted_lanelet_boundary_cell_array = std::move(iter[0][0]["predicted_lanelet_boundary"]);
				_predicted_lanelet_boundary.resize(n_vehicles());
				for (unsigned int i = 0; i < n_vehicles(); ++i) {
					matlab::data::TypedArray<double> left_lanelet_array = std::move(predicted_lanelet_boundary_cell_array[i][0]);
					_predicted_lanelet_boundary[i][0] = ColMajorMatrixAccessor<double>(std::vector<double>(left_lanelet_array.begin(), left_lanelet_array.end()), left_lanelet_array.getDimensions());

					matlab::data::TypedArray<double> right_lanelet_array = std::move(predicted_lanelet_boundary_cell_array[i][1]);
					_predicted_lanelet_boundary[i][1] = ColMajorMatrixAccessor<double>(std::vector<double>(right_lanelet_array.begin(), right_lanelet_array.end()), right_lanelet_array.getDimensions());
				}
			}

			matlab::data::CellArray vehicle_obstacles_cell_array = std::move(iter[0][0]["vehicle_obstacles"]);
			_vehicle_obstacles.resize(n_hp());
			for (unsigned int i = 0; i < n_hp(); ++i) {
				matlab::data::CellArray vehicle_obstacles_cell_array_step_i = std::move(vehicle_obstacles_cell_array[0][i]);
				auto n_obstacles = vehicle_obstacles_cell_array_step_i.getDimensions()[1];
				_vehicle_obstacles[i].resize(n_obstacles);
				for (unsigned int j = 0; j < n_obstacles; ++j) {
					matlab::data::TypedArray<double> vehicle_obstacle = std::move(vehicle_obstacles_cell_array_step_i[j]);
					_vehicle_obstacles[i][j] = ColMajorMatrixAccessor<double>(std::vector<double>(vehicle_obstacle.begin(), vehicle_obstacle.end()), vehicle_obstacle.getDimensions());
				}
			}

			// auto t0 = std::chrono::high_resolution_clock::now();

			// do graph search to find solution
			Node<FLOATING_POINT_TYPE> const *sol = find(x0, trim_indices);

			// auto t1 = std::chrono::high_resolution_clock::now();
			// Printer::println(std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count(), "ms");

			auto [next_nodes, predicted_trims, y_predicted, is_exhausted] = respond_matlab_centralized(sol);

			matlab::data::TypedArray<bool> is_exhausted_array = _factory.createScalar<bool>(std::move(is_exhausted));

			matlab::data::CellArray next_nodes_array = _factory.createCellArray({1, n_hp()});
			for (unsigned i = 0; i < n_hp(); ++i) {
				next_nodes_array[0][i] = _factory.createArray(next_nodes[i].get_dim<std::size_t>(), next_nodes[i].begin(), next_nodes[i].end());
			}

			auto predicted_trims_ = std::vector<double>(predicted_trims.begin(), predicted_trims.end());
			matlab::data::TypedArray<double> predicted_trims_array = _factory.createArray(predicted_trims.get_dim<std::size_t>(), predicted_trims_.begin(), predicted_trims_.end());

			matlab::data::CellArray y_predicted_array = _factory.createCellArray({1, n_vehicles()});
			for (unsigned int i = 0; i < n_vehicles(); ++i) {
				y_predicted_array[0][i] = _factory.createArray(y_predicted[i].get_dim<std::size_t>(), y_predicted[i].begin(), y_predicted[i].end());
			}

			matlab::data::TypedArray<uint32_t> n_expanded_array = _factory.createScalar<uint32_t>(std::move(get_n_expanded()));

			matlab::data::CellArray shapes_array = _factory.createCellArray({1, n_hp()});

			return {next_nodes_array, predicted_trims_array, y_predicted_array, n_expanded_array, is_exhausted_array};
		}

		std::tuple<matlab::data::CellArray, matlab::data::TypedArray<double>, matlab::data::CellArray, matlab::data::CellArray, matlab::data::TypedArray<uint32_t>, matlab::data::TypedArray<bool>> graph_search_callback_pb(
		    matlab::data::ObjectArray const &iter, const std::function<Node<FLOATING_POINT_TYPE> const *(ColMajorMatrixAccessor<FLOATING_POINT_TYPE> const &, std::vector<std::uint8_t> const &)> &find,
		    const std::function<uint64_t(void)> &get_n_expanded) {
			matlab::data::TypedArray<double> trim_indices_array = std::move(iter[0][0]["trim_indices"]);
			std::vector<std::uint8_t> trim_indices(trim_indices_array.begin(), trim_indices_array.end());

			for (auto &e : trim_indices) {
				e -= 1;
			}

			matlab::data::TypedArray<double> x0_array = std::move(iter[0][0]["x0"]);
			ColMajorMatrixAccessor<double> x0 = ColMajorMatrixAccessor<double>(std::vector<double>(x0_array.begin(), x0_array.end()), x0_array.getDimensions());

			matlab::data::TypedArray<double> reference_trajectory_points_array = std::move(iter[0][0]["reference_trajectory_points"]);
			_reference_trajectory_points = ColMajorTensorAccessor<double>(std::vector<double>(reference_trajectory_points_array.begin(), reference_trajectory_points_array.end()), reference_trajectory_points_array.getDimensions());

			matlab::data::TypedArray<double> v_ref_array = std::move(iter[0][0]["v_ref"]);
			_v_ref = ColMajorMatrixAccessor<double>(std::vector<double>(v_ref_array.begin(), v_ref_array.end()), v_ref_array.getDimensions());

			if (scenario_type() == SCENARIO_TYPE::CommonRoad) {
				matlab::data::CellArray predicted_lanelet_boundary_cell_array = std::move(iter[0][0]["predicted_lanelet_boundary"]);
				_predicted_lanelet_boundary.resize(n_vehicles());
				for (unsigned int i = 0; i < n_vehicles(); ++i) {
					matlab::data::TypedArray<double> left_lanelet_array = std::move(predicted_lanelet_boundary_cell_array[i][0]);
					_predicted_lanelet_boundary[i][0] = ColMajorMatrixAccessor<double>(std::vector<double>(left_lanelet_array.begin(), left_lanelet_array.end()), left_lanelet_array.getDimensions());

					matlab::data::TypedArray<double> right_lanelet_array = std::move(predicted_lanelet_boundary_cell_array[i][1]);
					_predicted_lanelet_boundary[i][1] = ColMajorMatrixAccessor<double>(std::vector<double>(right_lanelet_array.begin(), right_lanelet_array.end()), right_lanelet_array.getDimensions());
				}
			}

			matlab::data::CellArray vehicle_obstacles_cell_array = std::move(iter[0][0]["vehicle_obstacles"]);
			_vehicle_obstacles.resize(n_hp());
			for (unsigned int i = 0; i < n_hp(); ++i) {
				matlab::data::CellArray vehicle_obstacles_cell_array_step_i = std::move(vehicle_obstacles_cell_array[0][i]);
				auto n_obstacles = vehicle_obstacles_cell_array_step_i.getDimensions()[1];
				_vehicle_obstacles[i].resize(n_obstacles);
				for (unsigned int j = 0; j < n_obstacles; ++j) {
					matlab::data::TypedArray<double> vehicle_obstacle = std::move(vehicle_obstacles_cell_array_step_i[j]);
					_vehicle_obstacles[i][j] = ColMajorMatrixAccessor<double>(std::vector<double>(vehicle_obstacle.begin(), vehicle_obstacle.end()), vehicle_obstacle.getDimensions());
				}
			}

			// auto t0 = std::chrono::high_resolution_clock::now();

			// do graph search to find solution
			Node<FLOATING_POINT_TYPE> const *sol = find(x0, trim_indices);

			// auto t1 = std::chrono::high_resolution_clock::now();
			// Printer::println(std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count(), "ms");

			auto [next_nodes, predicted_trims, y_predicted, shapes, is_exhausted] = respond_matlab_pb(sol);

			matlab::data::TypedArray<bool> is_exhausted_array = _factory.createScalar<bool>(std::move(is_exhausted));

			matlab::data::CellArray next_nodes_array = _factory.createCellArray({1, n_hp()});
			for (unsigned i = 0; i < n_hp(); ++i) {
				next_nodes_array[0][i] = _factory.createArray(next_nodes[i].get_dim<std::size_t>(), next_nodes[i].begin(), next_nodes[i].end());
			}

			auto predicted_trims_ = std::vector<double>(predicted_trims.begin(), predicted_trims.end());
			matlab::data::TypedArray<double> predicted_trims_array = _factory.createArray(predicted_trims.get_dim<std::size_t>(), predicted_trims_.begin(), predicted_trims_.end());

			matlab::data::CellArray y_predicted_array = _factory.createCellArray({1, n_vehicles()});
			for (unsigned int i = 0; i < n_vehicles(); ++i) {
				y_predicted_array[0][i] = _factory.createArray(y_predicted[i].get_dim<std::size_t>(), y_predicted[i].begin(), y_predicted[i].end());
			}

			matlab::data::TypedArray<uint32_t> n_expanded_array = _factory.createScalar<uint32_t>(std::move(get_n_expanded()));

			matlab::data::CellArray shapes_array = _factory.createCellArray({1, n_hp()});
			if (is_pb() == true) {
				for (unsigned int i = 0; i < n_hp(); ++i) {
					shapes_array[0][i] = _factory.createArray(shapes[i].get_dim<std::size_t>(), shapes[i].begin(), shapes[i].end());
				}
			}

			return {next_nodes_array, predicted_trims_array, y_predicted_array, shapes_array, n_expanded_array, is_exhausted_array};
		}
	};
}  // namespace GraphBasedPlanning
