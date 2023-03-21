#pragma once

#include "GraphSearch.h"

namespace GraphBasedPlanning {
	class GraphSearchSpecialization : protected virtual GraphSearch {
	   protected:
		[[nodiscard]] inline virtual unsigned int& n_trims() = 0;
		[[nodiscard]] inline virtual unsigned int n_trims() const = 0;

		[[nodiscard]] inline virtual unsigned int& n_hp() = 0;
		[[nodiscard]] inline virtual unsigned int n_hp() const = 0;

		[[nodiscard]] inline virtual unsigned int& n_vehicles() = 0;
		[[nodiscard]] inline virtual unsigned int n_vehicles() const = 0;

	   private:
		virtual Node<FLOATING_POINT_TYPE> const* find_solution(ColMajorMatrixAccessor<FLOATING_POINT_TYPE> const& x0, std::vector<std::uint8_t> const& trim_indices) = 0;
		virtual void clean() = 0;
	};
}  // namespace GraphBasedPlanning