#pragma once

#include <cassert>
#include <vector>

#include "ColMajorAccessor.h"
#include "ConfigData.h"
#include "MexException.h"
#include "Printer.h"
#include "generator.h"

// the following is necessary, because you can't use overridden virtual template functions;
#ifdef GRAPH_SEARCH_FLOAT
typedef float FLOATING_POINT_TYPE;
#else
typedef double FLOATING_POINT_TYPE;
#endif

namespace GraphBasedPlanning {
	class MPA {
	   private:
		[[nodiscard]] inline virtual unsigned int &n_trims() = 0;
		[[nodiscard]] inline virtual unsigned int n_trims() const = 0;

		[[nodiscard]] inline virtual unsigned int &n_hp() = 0;
		[[nodiscard]] inline virtual unsigned int n_hp() const = 0;

		[[nodiscard]] inline virtual unsigned int &n_vehicles() = 0;
		[[nodiscard]] inline virtual unsigned int n_vehicles() const = 0;

	   protected:
		std::vector<std::vector<std::vector<std::uint8_t>>> _reachability_list;

		ColMajorTensorAccessor<std::uint8_t> _transition_matrix_single;
		ColMajorMatrixAccessor<FLOATING_POINT_TYPE> _transition_matrix_mean_speed;
		std::vector<uint16_t> _distance_to_equilibrium;

		bool mpa_initialized = false;

		void mpa_callback() {
			if (_transition_matrix_single.empty()) {
				throw MexException("Transition matrix not initialized!");
			}

			n_trims() = _transition_matrix_single.size_of_dimension(0);  // or 1
			n_hp() = _transition_matrix_single.size_of_dimension(2);

			_reachability_list = std::vector<std::vector<std::vector<uint8_t>>>(n_hp());

			for (uint8_t i = 0; i < n_hp(); ++i) {
				_reachability_list[i].resize(n_trims());
				for (uint8_t j = 0; j < n_trims(); ++j) {
					for (uint8_t k = 0; k < n_trims(); ++k) {
						if (_transition_matrix_single(j, k, i)) {
							_reachability_list[i][j].push_back(k);
						}
					}
				}
			}

		}

		[[nodiscard]] inline virtual std::vector<FLOATING_POINT_TYPE> const &xs(unsigned int from, unsigned int to) const = 0;
		[[nodiscard]] inline virtual std::vector<FLOATING_POINT_TYPE> const &ys(unsigned int from, unsigned int to) const = 0;
		[[nodiscard]] inline virtual std::vector<FLOATING_POINT_TYPE> const &yaws(unsigned int from, unsigned int to) const = 0;
		[[nodiscard]] inline virtual FLOATING_POINT_TYPE dx(unsigned int from, unsigned int to) const = 0;
		[[nodiscard]] inline virtual FLOATING_POINT_TYPE dy(unsigned int from, unsigned int to) const = 0;
		[[nodiscard]] inline virtual FLOATING_POINT_TYPE dyaw(unsigned int from, unsigned int to) const = 0;
		[[nodiscard]] inline virtual std::vector<FLOATING_POINT_TYPE> const &area(unsigned int from, unsigned int to) const = 0;
		[[nodiscard]] inline virtual std::vector<FLOATING_POINT_TYPE> const &area_without_offset(unsigned int from, unsigned int to) const = 0;
		[[nodiscard]] inline virtual std::vector<FLOATING_POINT_TYPE> const &area_large_offset(unsigned int from, unsigned int to) const = 0;

		[[nodiscard]] std::uint8_t *get_reachable_states_sizes(std::uint8_t const *const current_states, std::uint8_t const next_step) const {
			// assert(mpa_initialized);
			std::uint8_t *sizes = new std::uint8_t[n_vehicles()];
			for (unsigned int i = 0; i < n_vehicles(); ++i) {
				sizes[i] = _reachability_list[next_step - 1][current_states[i]].size();
			}
			return sizes;
		}

		[[nodiscard]] unsigned int *get_reachable_states_multipliers(std::uint8_t const *const sizes) const {
			// assert(mpa_initialized);
			unsigned int *multipliers = new unsigned int[n_vehicles() + 1];
			multipliers[0] = 1U;
			for (unsigned int i = 1; i <= n_vehicles(); ++i) {
				multipliers[i] = multipliers[i - 1] * sizes[i - 1];
			}

			return multipliers;
		}

		void reachable_states(std::uint8_t const *const current_states, std::uint8_t const *const sizes, unsigned int const *const multipliers, std::uint8_t const next_step, std::uint8_t *const &next_trims, unsigned int const i) const {
			// assert(mpa_initialized);
			for (unsigned int j = 0; j < n_vehicles(); ++j) {
				next_trims[j] = _reachability_list[next_step - 1][current_states[j]][i / multipliers[j] % sizes[j]];
			}
		}

		unique_generator<std::uint8_t *> reachable_states(std::uint8_t const *const current_states, std::uint8_t const *const sizes, unsigned int const *const multipliers, std::uint8_t const next_step) const {
			// assert(mpa_initialized);
			std::uint8_t *next_trims = new std::uint8_t[n_vehicles()];

			for (unsigned int i = 0; i < multipliers[n_vehicles()]; ++i) {
				for (unsigned int j = 0; j < n_vehicles(); ++j) {
					next_trims[j] = _reachability_list[next_step - 1][current_states[j]][i / multipliers[j] % sizes[j]];
				}

				co_yield next_trims;
			}

			delete[] next_trims;
			delete[] sizes;
			delete[] multipliers;
		}
	};
}  // namespace GraphBasedPlanning