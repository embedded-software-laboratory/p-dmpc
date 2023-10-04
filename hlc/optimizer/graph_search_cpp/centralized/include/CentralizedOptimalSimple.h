#pragma once

#include <OptimalNode.h>
#include <Eval.h>

#include <queue>
#include <vector>

#include "CentralizedGraphSearch.h"
#include <ConfigData.h>
#include <MPA.h>
#include <VehicleData.h>

namespace GraphBasedPlanning {
	// CentralizedOptimalSimple is similar to CentralizedOptimalPolymorphic, but don't use polymorphic memory resource.
	template <unsigned int n_vehicles, SCENARIO_TYPE scenario_type, bool memory_saving = false>
	class CentralizedOptimalSimple : public CentralizedGraphSearch<n_vehicles, scenario_type> {
		OptimalNode<n_vehicles> *_root = nullptr;
		std::priority_queue<OptimalNode<n_vehicles> *, std::vector<OptimalNode<n_vehicles> *>, typename OptimalNode<n_vehicles>::priority_queue_comparison> _pq;

	   public:
		CentralizedOptimalSimple(std::shared_ptr<GraphBasedPlanning::ConfigData const> const &config, std::shared_ptr<GraphBasedPlanning::MPA const> const &mpa, std::array<VehicleData<scenario_type>, n_vehicles> const &&vehicle_data)
		    : CentralizedGraphSearch<n_vehicles, scenario_type>(config, mpa, std::forward<decltype(vehicle_data)>(vehicle_data)) {}
		Node<n_vehicles> const *search(Node<n_vehicles> const root) final {
			_root = new OptimalNode<n_vehicles>(root);

			expand_node(_root);

			while (true) {
				OptimalNode<n_vehicles> *const current_node = _pq.top();
				_pq.pop();

				std::array<std::vector<vec2>, n_vehicles> vehicles_obstacles;
				if (current_node->k() < this->_config->n_hp()) {
					vehicles_obstacles = this->_mpa->template calc_vehicles_obstacles_without_offset<n_vehicles>(current_node);
				} else {
					vehicles_obstacles = this->_mpa->template calc_vehicles_obstacles_large_offset<n_vehicles>(current_node);
				}

				if (!this->is_path_valid(vehicles_obstacles)) {
					if constexpr (memory_saving) {
						current_node->delete_from_parent();

						delete current_node;
					}
					continue;
				}

				if (this->is_target_reached(current_node)) [[unlikely]] {
					return current_node;
				}

				expand_node(current_node);
			}
		}
		void clean() final {
#if DO_EVAL
			save(current_used_memory);
#endif
			delete this;  // commit suicide
		}

		~CentralizedOptimalSimple() { delete _root; }

	   private:
		void expand_node(OptimalNode<n_vehicles> *const node) {
			std::array<std::uint8_t, n_vehicles> const sizes = this->_mpa->template get_reachable_states_sizes<n_vehicles>(node->trims(), node->k());
			std::array<unsigned int, n_vehicles> const multipliers = this->_mpa->template get_reachable_states_multipliers<n_vehicles>(sizes);

			auto const n_children = multipliers.back() * sizes.back();

			node->create_children(n_children);

			for (auto i = 0; i < n_children; ++i) {
				auto const next_states = this->_mpa->template reachable_states<n_vehicles>(multipliers, i, node->trims(), sizes, node->k());

				auto *const new_node = new OptimalNode<n_vehicles>(node, node->g(), node->k() + 1);
				this->_mpa->template init_node<n_vehicles>(new_node, next_states);
				this->init_cost(new_node);

				node->child(i) = new_node;
				_pq.push(new_node);
			}
		}
	};
}  // namespace GraphBasedPlanning