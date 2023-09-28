#pragma once

#include <OptimalNode.h>

#include <memory_resource>
#include <queue>

#include "CentralizedGraphSearch.h"
#include "ConfigData.h"
#include "MPA.h"
#include "VehicleData.h"

namespace GraphBasedPlanning {
	// CentralizedOptimalPolymorphic is the C++ adaption of the matlab graph search.
	template <unsigned int n_vehicles, SCENARIO_TYPE scenario_type>
	class CentralizedOptimalPolymorphic : public CentralizedGraphSearch<n_vehicles, scenario_type> {
		std::pmr::monotonic_buffer_resource _upr;
		std::pmr::polymorphic_allocator<> _pa{&_upr};
		std::priority_queue<OptimalNode<n_vehicles> *, std::vector<OptimalNode<n_vehicles> *>, typename OptimalNode<n_vehicles>::priority_queue_comparison> _pq;

	   public:
		CentralizedOptimalPolymorphic(std::shared_ptr<GraphBasedPlanning::ConfigData const> const &config, std::shared_ptr<GraphBasedPlanning::MPA const> const &mpa, std::array<VehicleData<scenario_type>, n_vehicles> const &&vehicle_data)
		    : CentralizedGraphSearch<n_vehicles, scenario_type>(config, mpa, std::forward<decltype(vehicle_data)>(vehicle_data)) {}
		Node<n_vehicles> const *search(Node<n_vehicles> const root) final {
#if DO_EVAL
			std::vector<std::uint64_t> where_conflict(this->_config->n_hp(), 0);
			std::uint64_t n_expanded = 0;
#endif
			OptimalNode<n_vehicles> *_root = _pa.new_object<OptimalNode<n_vehicles>>(root);
			expand_node(_root);
#if DO_EVAL
			++n_expanded;
#endif
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
#if DO_EVAL
					++where_conflict[current_node->k() - 1];
#endif
					continue;
				}

				if (this->is_target_reached(current_node)) [[unlikely]] {
#if DO_EVAL
					save(where_conflict, where_conflict);
					save(n_expanded, n_expanded);
#endif
					return current_node;
				}

				expand_node(current_node);
#if DO_EVAL
				++n_expanded;
#endif
			}
		}
		void clean() final { delete this; }  // commit suicide, to uninitialize polymorphic data;

		~CentralizedOptimalPolymorphic() {
#if DO_EVAL
			save(current_used_memory);
#endif
		}

	   private:
		void expand_node(OptimalNode<n_vehicles> *const node) {
			std::array<std::uint8_t, n_vehicles> const sizes = this->_mpa->template get_reachable_states_sizes<n_vehicles>(node->trims(), node->k());
			std::array<unsigned int, n_vehicles> const multipliers = this->_mpa->template get_reachable_states_multipliers<n_vehicles>(sizes);

			auto const n_children = multipliers.back() * sizes.back();
			node->create_children(n_children, _pa);

			for (auto i = 0; i < n_children; ++i) {
				auto const next_states = this->_mpa->template reachable_states<n_vehicles>(multipliers, i, node->trims(), sizes, node->k());

				auto *const new_node = _pa.new_object<OptimalNode<n_vehicles>>(node, node->g(), node->k() + 1);
				this->_mpa->template init_node<n_vehicles>(new_node, next_states);
				this->init_cost(new_node);

				node->child(i) = new_node;
				_pq.push(new_node);
			}
		}
	};
}  // namespace GraphBasedPlanning