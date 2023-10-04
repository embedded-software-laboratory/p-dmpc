#pragma once

#include <VehicleObstaclesData.h>
#include <ConfigData.h>

#include <MatlabDataArray.hpp>
#include <mex.hpp>

namespace GraphBasedPlanning {
	VehicleObstaclesData make_vehicle_obstacles_data(matlab::data::ObjectArray const& iter, std::shared_ptr<ConfigData const> const& config, std::shared_ptr<matlab::engine::MATLABEngine>& _matlab) {
		VehicleObstaclesData ret;

		matlab::data::CellArray vehicle_obstacles_cell_array = std::move(iter[0][0]["vehicle_obstacles"]);
		ret.resize(config->n_hp());
		for (unsigned int i = 0; i < config->n_hp(); ++i) {
			matlab::data::CellArray vehicle_obstacles_cell_array_step_i = std::move(vehicle_obstacles_cell_array[0][i]);
			auto n_obstacles = vehicle_obstacles_cell_array_step_i.getDimensions()[1];
			ret[i].resize(n_obstacles);
			for (unsigned int j = 0; j < n_obstacles; ++j) {
				matlab::data::TypedArray<double> vehicle_obstacle = std::move(vehicle_obstacles_cell_array_step_i[j]);
				ret[i][j] = ColMajorMatrixAccessor<double>(std::vector<double>(vehicle_obstacle.begin(), vehicle_obstacle.end()), vehicle_obstacle.getDimensions());
			}
		}

		return ret;
	}
}  // namespace GraphBasedPlanning