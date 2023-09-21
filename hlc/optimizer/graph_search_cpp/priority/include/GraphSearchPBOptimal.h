#pragma once

#include <PriorityBasedNode.h>

#include <queue>
#include <vector>

#include "VehicleObstaclesData.h"
#include "ConfigData.h"
#include "MPA.h"
#include "PriorityBasedGraphSearch.h"
#include "VehicleData.h"

namespace GraphBasedPlanning {
	template <SCENARIO_TYPE scenario_type>
	class GraphSearchPBOptimal : public PriorityBasedGraphSearch<scenario_type> {
		PriorityBasedNode *_root = nullptr;
		uint64_t n_expanded = 1;
		std::priority_queue<PriorityBasedNode *, std::vector<PriorityBasedNode *>, typename PriorityBasedNode::priority_queue_comparison> _pq;

	   public:
		GraphSearchPBOptimal(std::shared_ptr<GraphBasedPlanning::ConfigData const> const &config, std::shared_ptr<GraphBasedPlanning::MPA const> const &mpa, std::array<VehicleData<scenario_type>, 1> const &&vehicle_data, VehicleObstaclesData const&& vehicle_obstacles_data)
		    : PriorityBasedGraphSearch<scenario_type>(config, mpa, std::forward<decltype(vehicle_data)>(vehicle_data), std::forward<decltype(vehicle_obstacles_data)>(vehicle_obstacles_data)) {}
		PriorityBasedNode const *search(PriorityBasedNode const root) final {
			_root = new PriorityBasedNode(root);

			expand_node(_root);

			while (true) {
				if (!_pq.empty()) {
					PriorityBasedNode *const current_node = _pq.top();
					_pq.pop();

					this->set_predicted_areas(current_node);

					if (!this->check_vehicle_obstacles(current_node)) {
						continue;
					}

					// TODO laneletcrossingarea against area without offset

					// TODO HDV Reachable sets against area normal offset

					std::array<std::vector<vec2>, 1> vehicles_obstacle;
					if (current_node->k() < this->_config->n_hp()) {
						vehicles_obstacle = this->_mpa->template calc_vehicles_obstacles_without_offset<1>(current_node);
					} else {
						vehicles_obstacle = this->_mpa->template calc_vehicles_obstacles_large_offset<1>(current_node);
					}

					if constexpr (scenario_type == SCENARIO_TYPE::CommonRoad) {
						if (!this->lanelet_interaction_valid(vehicles_obstacle[0], 0)) continue;
					}

					if (this->is_target_reached(current_node)) [[unlikely]] {
						return current_node;
					}

					++n_expanded;
					expand_node(current_node);
				} else {
					return nullptr;
				}
			}
		}

		void clean() final { delete this; }

		std::uint64_t get_n_expanded() final { return n_expanded; }

		~GraphSearchPBOptimal() { delete _root; }

	   private:
		void expand_node(PriorityBasedNode *const node) {
			std::array<std::uint8_t, 1> const sizes = this->_mpa->template get_reachable_states_sizes<1>(node->trims(), node->k());
			std::array<unsigned int, 1> const multipliers = this->_mpa->template get_reachable_states_multipliers<1>(sizes);

			auto const n_children = multipliers.back() * sizes.back();
			n_expanded += n_children;

			node->create_children(n_children);

			for (auto i = 0; i < n_children; ++i) {
				auto const next_states = this->_mpa->template reachable_states<1>(multipliers, i, node->trims(), sizes, node->k());

				auto *const new_node = new PriorityBasedNode(node, node->g(), node->k() + 1);
				this->_mpa->template init_node<1>(new_node, next_states);
				this->init_cost(new_node);

				node->child(i) = new_node;
				_pq.push(new_node);
			}
		}
	};
}  // namespace GraphBasedPlanning