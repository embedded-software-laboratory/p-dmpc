#pragma once

#include <vector>

#include "ColMajorAccessor.h"
#include "MPA.h"
#include "Maneuver.h"
#include "MatlabDataArray.hpp"
#include <utility>

namespace GraphBasedPlanning {
	class MPA_MEX : protected virtual MPA {
		ColMajorMatrixAccessor<Maneuver> _maneuvers;
		std::shared_ptr<matlab::engine::MATLABEngine> &_matlab;

	   protected:
		[[nodiscard]] inline std::vector<double> const &xs(unsigned int from, unsigned int to) const final { return _maneuvers(from, to).xs; }
		[[nodiscard]] inline std::vector<double> const &ys(unsigned int from, unsigned int to) const final { return _maneuvers(from, to).ys; }
		[[nodiscard]] inline std::vector<double> const &yaws(unsigned int from, unsigned int to) const final { return _maneuvers(from, to).yaws; }
		[[nodiscard]] inline double dx(unsigned int from, unsigned int to) const final { return _maneuvers(from, to).dx; }
		[[nodiscard]] inline double dy(unsigned int from, unsigned int to) const final { return _maneuvers(from, to).dy; }
		[[nodiscard]] inline double dyaw(unsigned int from, unsigned int to) const final { return _maneuvers(from, to).dyaw; }
		[[nodiscard]] inline std::vector<double> const &area(unsigned int from, unsigned int to) const final { return _maneuvers(from, to).area; }
		[[nodiscard]] inline std::vector<double> const &area_without_offset(unsigned int from, unsigned int to) const final { return _maneuvers(from, to).area_without_offset; }
		[[nodiscard]] inline std::vector<double> const &area_large_offset(unsigned int from, unsigned int to) const final { return _maneuvers(from, to).area_large_offset; }

		explicit MPA_MEX(std::shared_ptr<matlab::engine::MATLABEngine> &_matlab) : _matlab(_matlab) {}

		void mpa_callback(matlab::data::Array const &mpa_array) {
			matlab::data::TypedArray<double> transition_matrix_single_array = _matlab->getProperty(mpa_array, u"transition_matrix_single");
			_transition_matrix_single = ColMajorTensorAccessor(std::vector<std::uint8_t>(transition_matrix_single_array.begin(), transition_matrix_single_array.end()), transition_matrix_single_array.getDimensions());

			matlab::data::TypedArray<double> const transition_matrix_mean_speed_array = _matlab->getProperty(mpa_array, u"transition_matrix_mean_speed");
			_transition_matrix_mean_speed = ColMajorMatrixAccessor(std::vector<double>(transition_matrix_mean_speed_array.begin(), transition_matrix_mean_speed_array.end()), transition_matrix_mean_speed_array.getDimensions());

			matlab::data::TypedArray<uint16_t> distance_to_equilibrium_array = _matlab->getProperty(mpa_array, u"distance_to_equilibrium");
			_distance_to_equilibrium.assign(distance_to_equilibrium_array.begin(), distance_to_equilibrium_array.end());
			
			MPA::mpa_callback();

			matlab::data::CellArray const maneuvers_array = _matlab->getProperty(mpa_array, u"maneuvers");
			std::vector<Maneuver> maneuvers(maneuvers_array.getNumberOfElements());
			for (unsigned int i = 0; i < maneuvers_array.getNumberOfElements(); ++i) {
				if (!(maneuvers_array.cbegin() + i)->isEmpty()) {
					matlab::data::StructArray const maneuver_array = std::move(*(maneuvers_array.cbegin() + i));

					matlab::data::TypedArray<double> const xs_array = std::move(maneuver_array[0][0]["xs"]);
					matlab::data::TypedArray<double> const ys_array = std::move(maneuver_array[0][0]["ys"]);
					matlab::data::TypedArray<double> const yaws_array = std::move(maneuver_array[0][0]["yaws"]);
					matlab::data::TypedArray<double> const dx_array = std::move(maneuver_array[0][0]["dx"]);
					matlab::data::TypedArray<double> const dy_array = std::move(maneuver_array[0][0]["dy"]);
					matlab::data::TypedArray<double> const dyaw_array = std::move(maneuver_array[0][0]["dyaw"]);
					matlab::data::TypedArray<double> const area_array = std::move(maneuver_array[0][0]["area"]);
					matlab::data::TypedArray<double> const area_without_offset_array = std::move(maneuver_array[0][0]["area_without_offset"]);
					matlab::data::TypedArray<double> const area_large_offset_array = std::move(maneuver_array[0][0]["area_large_offset"]);

					maneuvers[i].xs = std::move(std::vector<double>(xs_array.begin(), xs_array.end()));
					maneuvers[i].ys = std::move(std::vector<double>(ys_array.begin(), ys_array.end()));
					maneuvers[i].yaws = std::move(std::vector<double>(yaws_array.begin(), yaws_array.end()));
					maneuvers[i].dx = dx_array[0][0];
					maneuvers[i].dy = dy_array[0][0];
					maneuvers[i].dyaw = dyaw_array[0][0];
					maneuvers[i].area = ColMajorMatrixAccessor<double>(std::vector<double>(area_array.begin(), area_array.end()), area_array.getDimensions());
					maneuvers[i].area_without_offset = ColMajorMatrixAccessor<double>(std::vector<double>(area_without_offset_array.begin(), area_without_offset_array.end()), area_without_offset_array.getDimensions());
					maneuvers[i].area_large_offset = ColMajorMatrixAccessor<double>(std::vector<double>(area_large_offset_array.begin(), area_large_offset_array.end()), area_large_offset_array.getDimensions());
				}
			}
			_maneuvers = ColMajorMatrixAccessor(std::move(maneuvers), maneuvers_array.getDimensions());

			mpa_initialized = true;
		}
	};
}  // namespace GraphBasedPlanning