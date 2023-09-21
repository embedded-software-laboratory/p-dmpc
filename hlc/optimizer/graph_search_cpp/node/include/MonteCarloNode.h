#pragma once

#include <array>
#include <concepts>
#include <cstdint>

#include "Node.h"

namespace GraphBasedPlanning {
	template <unsigned int n_vehicles>
	class MonteCarloNode : public Node<n_vehicles> {
		//double _h;
	   public:
		MonteCarloNode(std::array<std::uint8_t, n_vehicles> const&& trims, std::array<double, n_vehicles> const&& xs, std::array<double, n_vehicles> const&& ys, std::array<double, n_vehicles> const&& yaws)
		    : Node<n_vehicles>(std::forward<decltype(trims)>(trims), std::forward<decltype(xs)>(xs), std::forward<decltype(ys)>(ys), std::forward<decltype(yaws)>(yaws)) {} //, _h(0.0) {}

		MonteCarloNode(MonteCarloNode* const parent_, double const g_, std::uint8_t const k_) : Node<n_vehicles>(std::forward<decltype(parent_)>(parent_), std::forward<decltype(g_)>(g_), std::forward<decltype(k_)>(k_)) {} //, _h(0.0) {}

		//double& h() { return _h; }  // cost to go
		//[[nodiscard]] double h() const { return _h; }

		explicit MonteCarloNode(Node<n_vehicles> root) : Node<n_vehicles>(root) {}

		struct priority_queue_comparison {
			bool operator()(MonteCarloNode const* const lhs, MonteCarloNode const* const rhs) const { return lhs->g() > rhs->g(); } //{ return lhs->g() + lhs->h() > rhs->g() + rhs->h(); }
		};
	};
}  // namespace GraphBasedPlanning