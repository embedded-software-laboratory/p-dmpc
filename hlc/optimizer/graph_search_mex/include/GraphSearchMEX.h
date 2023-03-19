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

		matlab::data::ArrayFactory &_factory;
		std::shared_ptr<matlab::engine::MATLABEngine> &_matlab;

	   protected:
		GraphSearchMEX(matlab::data::ArrayFactory &factory, std::shared_ptr<matlab::engine::MATLABEngine> &_matlab) : MPA_MEX(_matlab), _factory(factory), _matlab(_matlab) {}

		void scenario_callback(matlab::data::Array const &scenario) {
			matlab::data::Array const mpa_array = _matlab->getProperty(scenario, u"mpa");
			mpa_callback(mpa_array);

			matlab::data::Array const options_array = _matlab->getProperty(scenario, u"options");
			matlab::data::TypedArray<double> const tick_per_step_array = _matlab->getProperty(options_array, u"tick_per_step");
			tick_per_step() = static_cast<unsigned int>(tick_per_step_array[0][0]);
			matlab::data::TypedArray<double> const dt_array = _matlab->getProperty(options_array, u"dt");
			dt() = dt_array[0][0];

			matlab::data::Array const vehicles_array = _matlab->getProperty(scenario, u"vehicles");
			n_vehicles() = vehicles_array.getNumberOfElements();
			Node<FLOATING_POINT_TYPE>::init_static_variables(n_vehicles());

			reference_trajectory().resize(n_vehicles());
			for (unsigned int i = 0; i < n_vehicles(); ++i) {
				matlab::data::TypedArray<double> const reference_trajectory_array = _matlab->getProperty(vehicles_array, i, u"referenceTrajectory");
				reference_trajectory()[i].resize(reference_trajectory_array.getDimensions()[0]);

				// blöd, dass das Format in Matlab schlecht gewählt ist, sodass das folgende nötig wird:
				for (unsigned int j = 0; j < reference_trajectory()[i].size(); ++j) {
					reference_trajectory()[i][j].x = reference_trajectory_array[j][0];
					reference_trajectory()[i][j].y = reference_trajectory_array[j][1];
				}
			}

			matlab::data::CharArray const scenario_name_array = _matlab->getProperty(options_array, u"scenario_name");
			if (scenario_name_array.toAscii() == "Commonroad") {
				scenario_type() = SCENARIO_TYPE::CommonRoad;
			} else if (scenario_name_array.toAscii().find("circle") != std::string::npos) {
				scenario_type() = SCENARIO_TYPE::Circle;
			} else {
				throw MexException("Scenario type '", scenario_name_array.toAscii(), "' not implemented/available!");
			}
		}

		std::tuple<matlab::data::TypedArray<double>, matlab::data::TypedArray<double>, matlab::data::CellArray> graph_search_callback(
		    matlab::data::Array const &iter, const std::function<Node<FLOATING_POINT_TYPE> const *(ColMajorMatrixAccessor<FLOATING_POINT_TYPE> const &, std::vector<std::uint8_t> const &)> &find) {
			matlab::data::TypedArray<double> const trim_indices_array = _matlab->getProperty(iter, u"trim_indices");
			std::vector<std::uint8_t> trim_indices(trim_indices_array.begin(), trim_indices_array.end());

			for (auto &e : trim_indices) {
				e -= 1;
			}

			matlab::data::TypedArray<double> const x0_array = _matlab->getProperty(iter, u"x0");
			ColMajorMatrixAccessor<double> const x0 = ColMajorMatrixAccessor<double>(std::vector<double>(x0_array.begin(), x0_array.end()), x0_array.getDimensions());

			matlab::data::TypedArray<double> const reference_trajectory_points_array = _matlab->getProperty(iter, u"referenceTrajectoryPoints");
			_reference_trajectory_points = ColMajorTensorAccessor<double>(std::vector<double>(reference_trajectory_points_array.begin(), reference_trajectory_points_array.end()), reference_trajectory_points_array.getDimensions());

			matlab::data::TypedArray<double> const v_ref_array = _matlab->getProperty(iter, u"v_ref");
			_v_ref = ColMajorMatrixAccessor<double>(std::vector<double>(v_ref_array.begin(), v_ref_array.end()), v_ref_array.getDimensions());

			if (scenario_type() == SCENARIO_TYPE::CommonRoad) {
				matlab::data::CellArray const predicted_lanelet_boundary_cell_array = _matlab->getProperty(iter, u"predicted_lanelet_boundary");
				_predicted_lanelet_boundary.resize(n_vehicles());
				for (unsigned int i = 0; i < n_vehicles(); ++i) {
					matlab::data::TypedArray<double> const left_lanelet_array = std::move(predicted_lanelet_boundary_cell_array[i][0]);
					_predicted_lanelet_boundary[i][0] = ColMajorMatrixAccessor<double>(std::vector<double>(left_lanelet_array.begin(), left_lanelet_array.end()), left_lanelet_array.getDimensions());

					matlab::data::TypedArray<double> const right_lanelet_array = std::move(predicted_lanelet_boundary_cell_array[i][1]);
					_predicted_lanelet_boundary[i][1] = ColMajorMatrixAccessor<double>(std::vector<double>(right_lanelet_array.begin(), right_lanelet_array.end()), right_lanelet_array.getDimensions());
				}
			}

			auto t0 = std::chrono::high_resolution_clock::now();

			// do A*
			Node<FLOATING_POINT_TYPE> const *sol = find(x0, trim_indices);

			auto t1 = std::chrono::high_resolution_clock::now();
			Printer::println(std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count(), "ms");

			auto [next_node, predicted_trims, y_predicted] = respond_matlab(sol);

			matlab::data::TypedArray<double> next_node_array = _factory.createArray(next_node.get_dim<std::size_t>(), next_node.begin(), next_node.end());

			auto predicted_trims_ = std::vector<double>(predicted_trims.begin(), predicted_trims.end());
			matlab::data::TypedArray<double> predicted_trims_array = _factory.createArray(predicted_trims.get_dim<std::size_t>(), predicted_trims_.begin(), predicted_trims_.end());

			matlab::data::CellArray y_predicted_array = _factory.createCellArray({1, n_vehicles()});
			for (unsigned int i = 0; i < n_vehicles(); ++i) {
				y_predicted_array[0][i] = _factory.createArray(y_predicted[i].get_dim<std::size_t>(), y_predicted[i].begin(), y_predicted[i].end());
			}

			return {next_node_array, predicted_trims_array, y_predicted_array};
		}
	};
}  // namespace GraphBasedPlanning
