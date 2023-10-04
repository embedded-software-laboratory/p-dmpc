#pragma once

#include <array>
#include <concepts>
#include <cstdint>

#include "Node.h"

namespace GraphBasedPlanning {
	template <unsigned int n_vehicles>
	class RandomSimulationNode : public Node<n_vehicles> {
		bool _last = true;

	   public:
		RandomSimulationNode(Node<n_vehicles>* const parent_, double const g_, std::uint8_t const k_, bool last) : Node<n_vehicles>(std::forward<decltype(parent_)>(parent_), std::forward<decltype(g_)>(g_), std::forward<decltype(k_)>(k_)), _last(last) {}

		~RandomSimulationNode() final {
			if(!_last) {
				delete Node<n_vehicles>::_parent;
			}
		}
	};
}  // namespace GraphBasedPlanning