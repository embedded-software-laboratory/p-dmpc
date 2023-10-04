#pragma once

#include <ConfigData.h>
#include <MatlabException.h>

#include <MatlabDataArray.hpp>
#include <mex.hpp>

namespace GraphBasedPlanning {
	ConfigData make_config(matlab::data::Array const &scenario, matlab::data::Array const &mpa_array, std::shared_ptr<matlab::engine::MATLABEngine> &_matlab) {
		// get the number of steps in the horizon prediction
		matlab::data::Array const options_array = _matlab->getProperty(scenario, u"options");
		matlab::data::TypedArray<double> const hp_array = _matlab->getProperty(options_array, u"Hp");
		unsigned int const n_hp = static_cast<unsigned int>(hp_array[0][0]);

		matlab::data::TypedArray<double> const tick_per_step_array = _matlab->getProperty(options_array, u"tick_per_step");
		unsigned int const tick_per_step = static_cast<unsigned int>(tick_per_step_array[0][0]);

		// get the time difference of the iterations
		matlab::data::TypedArray<double> const dt_array = _matlab->getProperty(options_array, u"dt_seconds");
		double const dt = dt_array[0][0];

		// get the number of vehicles
		matlab::data::Array const vehicles_array = _matlab->getProperty(scenario, u"vehicles");
		unsigned int const n_vehicles = vehicles_array.getNumberOfElements();

		// get the scenario type of the graph search
		matlab::data::EnumArray const scenario_type_array = _matlab->getProperty(options_array, u"scenario_type");
		std::string scenario_type_string = scenario_type_array[0][0];
		SCENARIO_TYPE scenario_type = SCENARIO_TYPE::Error;
		if (scenario_type_string == "commonroad") {
			scenario_type = SCENARIO_TYPE::CommonRoad;
		} else if (scenario_type_string == "circle") {
			scenario_type = SCENARIO_TYPE::Circle;
		} else {
			throw MatlabException("Scenario type '", scenario_type_string, "' not implemented/available!");
		}

		// get the reference trajectory of each vehicle
		std::vector<std::vector<vec2>> reference_trajectory(n_vehicles);
		for (auto i = 0; i < n_vehicles; ++i) {
			matlab::data::TypedArray<double> const reference_trajectory_array = _matlab->getProperty(vehicles_array, i, u"reference_trajectory");
			reference_trajectory[i].resize(reference_trajectory_array.getDimensions()[0]);

			// blöd, dass das Format in Matlab schlecht gewählt ist, sodass das folgende nötig wird:
			for (auto j = 0; j < reference_trajectory[i].size(); ++j) {
				reference_trajectory[i][j].x = reference_trajectory_array[j][0];
				reference_trajectory[i][j].y = reference_trajectory_array[j][1];
			}
		}

		// get number of trims in the MPA
		matlab::data::CellArray const maneuvers_array = _matlab->getProperty(mpa_array, u"maneuvers");
		unsigned int const n_trims = maneuvers_array.getDimensions()[0];

		matlab::data::TypedArray<bool> recursive_feasibility_array = _matlab->getProperty(options_array, u"recursive_feasibility");
		bool recursive_feasibility = static_cast<bool>(recursive_feasibility_array[0][0]);

		return {n_hp, n_vehicles, n_trims, tick_per_step, dt, scenario_type, std::move(reference_trajectory), recursive_feasibility};
	}
}  // namespace GraphBasedPlanning