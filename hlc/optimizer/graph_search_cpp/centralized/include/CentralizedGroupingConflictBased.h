#pragma once

#include <MatlabException.h>
#include <OptimalNode.h>

#include <memory_resource>
#include <queue>

#include "CentralizedGraphSearch.h"
#include "ConfigData.h"
#include "MPA.h"
#include "VehicleData.h"

namespace GraphBasedPlanning {
	template <unsigned int n_vehicles, SCENARIO_TYPE scenario_type>
	class CentralizedGroupingConflictBased : public CentralizedGraphSearch<n_vehicles, scenario_type> {
		std::pmr::monotonic_buffer_resource _upr;
		std::pmr::polymorphic_allocator<> _pa{&_upr};

	   public:
		CentralizedGroupingConflictBased(
		    std::shared_ptr<GraphBasedPlanning::ConfigData const> const &config, std::shared_ptr<GraphBasedPlanning::MPA const> const &mpa, std::array<VehicleData<scenario_type>, n_vehicles> const &&vehicle_data)
		    : CentralizedGraphSearch<n_vehicles, scenario_type>(config, mpa, std::forward<decltype(vehicle_data)>(vehicle_data)) {}
		Node<n_vehicles> const *search(Node<n_vehicles> const root) final {
#if DO_EVAL
			auto t2 = std::chrono::high_resolution_clock::now();
#endif
			std::array<std::priority_queue<OptimalNode<1> *, std::vector<OptimalNode<1> *>, typename OptimalNode<1>::priority_queue_comparison>, n_vehicles> pqs;

			auto produce_roots = []<std::size_t... I>(Node<n_vehicles> const &root, std::pmr::polymorphic_allocator<> &pa, std::index_sequence<I...>) -> std::array<OptimalNode<1> *, n_vehicles> {
				return {pa.new_object<OptimalNode<1>>(OptimalNode<1>({root.trim(I)}, {root.x(I)}, {root.y(I)}, {root.yaw(I)}))...};
			};

			std::array<OptimalNode<1> *, n_vehicles> roots = produce_roots(root, _pa, std::make_index_sequence<n_vehicles>{});

			std::array<Node<1> *, n_vehicles> minima;
			for (auto i = 0; i < n_vehicles; ++i) {
				expand_node(roots[i], pqs[i], i);

				while (true) {
					OptimalNode<1> *const current_node = pqs[i].top();
					pqs[i].pop();

					if constexpr (scenario_type != SCENARIO_TYPE::Circle) {
						std::array<std::vector<vec2>, 1> vehicles_obstacles;
						if (current_node->k() < CentralizedGraphSearch<n_vehicles, scenario_type>::_config->n_hp()) {
							vehicles_obstacles = CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->template calc_vehicles_obstacles_without_offset<1>(current_node);
						} else {
							vehicles_obstacles = CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->template calc_vehicles_obstacles_large_offset<1>(current_node);
						}

						if (!this->lanelet_interaction_valid(vehicles_obstacles.front(), i)) {
							continue;
						}
					}

					if (current_node->k() >= CentralizedGraphSearch<n_vehicles, scenario_type>::_config->n_hp()) [[unlikely]] {
						minima[i] = current_node;
						break;
					}

					expand_node(current_node, pqs[i], i);
				}
			}
			Node<n_vehicles> *solution = convert_nodes_to_single(minima);
#if DO_EVAL
			auto t3 = std::chrono::high_resolution_clock::now();
			auto elapsed = t3 - t2;
			double seconds = std::chrono::duration<double>(elapsed).count();

			save(time_grouping_conflict, seconds);
#endif
			if (this->check_path(solution)) {
#if DO_EVAL
				save(current_used_memory);
#endif
				return solution;
			}

			throw NoSolutionException();
		}
		void clean() final {
			delete this;
		}  // commit suicide, to uninitialize polymorphic data;

		~CentralizedGroupingConflictBased() = default;

	   private:
		void expand_node(OptimalNode<1> *const node, std::priority_queue<OptimalNode<1> *, std::vector<OptimalNode<1> *>, typename OptimalNode<1>::priority_queue_comparison> &pq, std::integral auto index) {
			std::array<std::uint8_t, 1> const sizes = this->_mpa->template get_reachable_states_sizes<1>(node->trims(), node->k());
			std::array<unsigned int, 1> const multipliers = this->_mpa->template get_reachable_states_multipliers<1>(sizes);

			auto const n_children = multipliers.back() * sizes.back();
			node->create_children(n_children, _pa);

			for (auto i = 0; i < n_children; ++i) {
				auto const next_states = this->_mpa->template reachable_states<1>(multipliers, i, node->trims(), sizes, node->k());

				auto *const new_node = _pa.new_object<OptimalNode<1>>(node, node->g(), node->k() + 1);
				this->_mpa->template init_node<1>(new_node, next_states);
				this->init_cost(new_node, index);

				node->child(i) = new_node;
				pq.push(new_node);
			}
		}

		Node<n_vehicles> *convert_nodes_to_single(std::array<Node<1> *, n_vehicles> const nodes) { return convert_nodes_to_single_impl(nodes, this->_config->n_hp(), std::make_integer_sequence<unsigned int, n_vehicles>{}); }

		template <unsigned int... I>
		Node<n_vehicles> *convert_nodes_to_single_impl(std::array<Node<1> *, n_vehicles> const nodes, int const k, std::integer_sequence<unsigned int, I...> &&Indices) {
			if (k < 0) {
				return nullptr;
			}

			return new Node<n_vehicles>(convert_nodes_to_single_impl({nodes[I]->parent()...}, k - 1, std::move(Indices)), k, {nodes[I]->trim(0 + I * 0)...}, {nodes[I]->x(0 + I * 0)...}, {nodes[I]->y(0 + I * 0)...},
			    {nodes[I]->yaw(0 + I * 0)...}, (nodes[I]->g() + ...));
		}
	};
}  // namespace GraphBasedPlanning