#pragma once

#include <ConfigData.h>
#include <MPA.h>

#include <MatlabDataArray.hpp>
#include <memory>
#include <mex.hpp>

namespace GraphBasedPlanning {
	MPA make_mpa(matlab::data::Array const &scenario, std::shared_ptr<ConfigData const> const &config, std::shared_ptr<matlab::engine::MATLABEngine> &_matlab) {
		matlab::data::Array const mpa_array = _matlab->getProperty(scenario, u"mpa");

		matlab::data::CellArray const maneuvers_array = _matlab->getProperty(mpa_array, u"maneuvers");

		// get number of trims in the MPA
		unsigned int const n_trims = maneuvers_array.getDimensions()[0];
		std::vector<std::vector<Maneuver<double>>> maneuvers(n_trims, std::vector<Maneuver<double>>(n_trims));
		for (auto i = 0; i < n_trims; ++i) {
			for (auto j = 0; j < n_trims; ++j) {
				if (!maneuvers_array[i][j].isEmpty()) {
					matlab::data::StructArray const maneuver_array = std::move(maneuvers_array[i][j]);

					matlab::data::TypedArray<double> const xs_array = std::move(maneuver_array[0][0]["xs"]);
					matlab::data::TypedArray<double> const ys_array = std::move(maneuver_array[0][0]["ys"]);
					matlab::data::TypedArray<double> const yaws_array = std::move(maneuver_array[0][0]["yaws"]);
					matlab::data::TypedArray<double> const dx_array = std::move(maneuver_array[0][0]["dx"]);
					matlab::data::TypedArray<double> const dy_array = std::move(maneuver_array[0][0]["dy"]);
					matlab::data::TypedArray<double> const dyaw_array = std::move(maneuver_array[0][0]["dyaw"]);
					matlab::data::TypedArray<double> const area_array = std::move(maneuver_array[0][0]["area"]);
					matlab::data::TypedArray<double> const area_without_offset_array = std::move(maneuver_array[0][0]["area_without_offset"]);
					matlab::data::TypedArray<double> const area_large_offset_array = std::move(maneuver_array[0][0]["area_large_offset"]);

					maneuvers[i][j].xs.assign(xs_array.begin(), xs_array.end());
					maneuvers[i][j].ys.assign(ys_array.begin(), ys_array.end());
					maneuvers[i][j].yaws.assign(yaws_array.begin(), yaws_array.end());
					maneuvers[i][j].dx = dx_array[0][0];
					maneuvers[i][j].dy = dy_array[0][0];
					maneuvers[i][j].dyaw = dyaw_array[0][0];
					maneuvers[i][j].area = ColMajorMatrixAccessor<double>(std::vector<double>(area_array.begin(), area_array.end()), area_array.getDimensions());
					maneuvers[i][j].area_without_offset = ColMajorMatrixAccessor<double>(std::vector<double>(area_without_offset_array.begin(), area_without_offset_array.end()), area_without_offset_array.getDimensions());
					maneuvers[i][j].area_large_offset = ColMajorMatrixAccessor<double>(std::vector<double>(area_large_offset_array.begin(), area_large_offset_array.end()), area_large_offset_array.getDimensions());
				}
			}
		}

		matlab::data::TypedArray<double> const transition_matrix_single_array = _matlab->getProperty(mpa_array, u"transition_matrix_single");
		std::vector<std::vector<std::vector<std::uint8_t>>> transition_matrix_single(config->n_hp());
		std::fill(transition_matrix_single.begin(), transition_matrix_single.end(), std::vector<std::vector<std::uint8_t>>(n_trims, std::vector<std::uint8_t>(n_trims)));
		for (auto i = 0; i < config->n_hp(); ++i) {
			for (auto j = 0; j < n_trims; ++j) {
				for (auto k = 0; k < n_trims; ++k) {
					transition_matrix_single[i][j][k] = static_cast<std::uint8_t>(transition_matrix_single_array[j][k][i]);
				}
			}
		}

		matlab::data::TypedArray<double> const transition_matrix_mean_speed_array = _matlab->getProperty(mpa_array, u"transition_matrix_mean_speed");
		std::vector<std::vector<double>> transition_matrix_mean_speed(n_trims, std::vector<double>(n_trims));
		for (auto i = 0; i < n_trims; ++i) {
			for (auto j = 0; j < n_trims; ++j) {
				transition_matrix_mean_speed[i][j] = transition_matrix_mean_speed_array[i][j];
			}
		}

		std::vector<std::vector<std::vector<std::uint8_t>>> reachability_list(config->n_hp());
		std::fill(reachability_list.begin(), reachability_list.end(), std::vector<std::vector<std::uint8_t>>(n_trims));
		for (auto i = 0; i < config->n_hp(); ++i) {
			for (auto j = 0; j < n_trims; ++j) {
				for (auto k = 0; k < n_trims; ++k) {
					if (transition_matrix_single[i][j][k]) {
						reachability_list[i][j].push_back(k);
					}
				}
			}
		}

		matlab::data::TypedArray<uint16_t> distance_to_equilibrium_array = _matlab->getProperty(mpa_array, u"distance_to_equilibrium");
		std::vector<std::uint16_t> distance_to_equilibrium(distance_to_equilibrium_array.begin(), distance_to_equilibrium_array.end());

		return {reachability_list, transition_matrix_single, std::move(transition_matrix_mean_speed), std::move(maneuvers), std::move(distance_to_equilibrium)};
	}
}  // namespace GraphBasedPlanning