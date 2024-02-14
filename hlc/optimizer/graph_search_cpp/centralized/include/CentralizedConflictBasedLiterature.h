#pragma once

#include <OptimalNode.h>

#include <array>
#include <numbers>
#include <queue>
#include <tuple>

#include "CentralizedGraphSearch.h"
#include "CentralizedOptimalSimple.h"
#include "ConfigData.h"
#include "MPA.h"
#include "VehicleData.h"

namespace GraphBasedPlanning {
	// CentralizedConflictBasedLiterature is the adaptation to our problem of the following reference:
	// Sharon, G., Stern, R., Felner, A., & Sturtevant, N. R. (2015). Conflict-based search for optimal multi-agent pathfinding. Artificial Intelligence, 219, 40-66.
	template <unsigned int n_vehicle>
	struct ConflictBasedNode {
		ConflictBasedNode const *_parent = nullptr;
		std::array<ConflictBasedNode *, n_vehicle> _children = {nullptr};
		std::vector<vec2> _collision;
		int _k = 0;
		int _i = -1;

		ConflictBasedNode(ConflictBasedNode const *const parent, std::vector<vec2> const &collision, int k, int i) : _parent(parent), _collision(collision), _k(k), _i(i) {}
		ConflictBasedNode() {}
		~ConflictBasedNode() {
			for (auto e : _children) delete e;
		}
	};

	template <unsigned int n_vehicles, SCENARIO_TYPE scenario_type>
	class CentralizedConflictBasedLiterature : public CentralizedGraphSearch<n_vehicles, scenario_type> {
		std::vector<OptimalNode<n_vehicles> *> _mins;
		ConflictBasedNode<n_vehicles> *conflict_based_root = new ConflictBasedNode<n_vehicles>();
		std::uint64_t branches = 0;
		std::uint64_t branches_feasible = 0;

	   public:
		CentralizedConflictBasedLiterature(
		    std::shared_ptr<GraphBasedPlanning::ConfigData const> const &config, std::shared_ptr<GraphBasedPlanning::MPA const> const &mpa, std::array<VehicleData<scenario_type>, n_vehicles> const &&vehicle_data)
		    : CentralizedGraphSearch<n_vehicles, scenario_type>(config, mpa, std::forward<decltype(vehicle_data)>(vehicle_data)) {}

		void dfs(std::array<OptimalNode<1>, n_vehicles> roots, ConflictBasedNode<n_vehicles> *conflict_based_node) {
			++branches;
			std::array<OptimalNode<1>, n_vehicles> roots_clone = roots;
			std::array<OptimalNode<1> *, n_vehicles> mins = {nullptr};
			std::array<Node<1> *, n_vehicles> mins_node = {nullptr};

			for (auto i = 0; i < n_vehicles; ++i) {
				std::priority_queue<OptimalNode<1> *, std::vector<OptimalNode<1> *>, typename OptimalNode<1>::priority_queue_comparison> pq;

				expand_node(&roots[i], i, pq);

				while (true) {
					if (pq.empty()) return;

					OptimalNode<1> *const current_node = pq.top();
					pq.pop();

					std::vector<vec2> vehicles_obstacle;
					if (current_node->k() < this->_config->n_hp()) {
						vehicles_obstacle = this->_mpa->calc_vehicles_obstacles_without_offset_conflict(current_node);
					} else {
						vehicles_obstacle = this->_mpa->calc_vehicles_obstacles_large_offset_conflict(current_node);
					}

					if (!is_path_valid_conflict_literature(vehicles_obstacle, i, current_node->k(), conflict_based_node)) {
						continue;
					}

					if (current_node->k() >= this->_config->n_hp()) [[unlikely]] {
						mins[i] = current_node;
						mins_node[i] = current_node;
						break;
					}

					expand_node(current_node, i, pq);
				}
			}

			if (check_path(mins_node, conflict_based_node)) {
				_mins.push_back(convert_nodes(mins));
				++branches_feasible;
			} else {
				for (auto e : conflict_based_node->_children) {
					if (e) {
						dfs(roots_clone, e);
					}
				}
			}
		}

		Node<n_vehicles> const *search(Node<n_vehicles> const root) final {
			auto produce_roots = [root]<std::size_t... I>(std::index_sequence<I...>) -> std::array<OptimalNode<1>, n_vehicles> { return {OptimalNode<1>({root.trim(I)}, {root.x(I)}, {root.y(I)}, {root.yaw(I)})...}; };
			dfs(produce_roots(std::make_index_sequence<n_vehicles>{}), conflict_based_root);

			if (_mins.empty()) throw MatlabException("not feasible!");
#if DO_EVAL
			save(current_used_memory);
#endif

			auto current_node = *std::min_element(_mins.begin(), _mins.end(), [](OptimalNode<n_vehicles> const *const lhs, OptimalNode<n_vehicles> const *const rhs) { return lhs->g() + lhs->h() < rhs->g() + rhs->h(); });

#if DO_EVAL
			save(branches, branches);
			save(branches_feasible, branches_feasible);
#endif
			return current_node;
		}
		void clean() final {
			delete this;  // commit suicide
		}

		~CentralizedConflictBasedLiterature() { delete conflict_based_root; }

	   private:
		[[nodiscard]] bool is_path_valid_conflict_literature(std::vector<vec2> const &vehicles_obstacle, std::integral auto const i, std::integral auto const k, ConflictBasedNode<n_vehicles> const *conflict_based_node) const {
			while (conflict_based_node->_parent) {
				if (i != conflict_based_node->_i && k == conflict_based_node->_k) {
					if (SAT::check_collision((vec2 const *)vehicles_obstacle.data(), vehicles_obstacle.size() - 1, (vec2 const *)conflict_based_node->_collision.data(), conflict_based_node->_collision.size() - 1)) return false;
				}
				conflict_based_node = conflict_based_node->_parent;
			}
			// lanelet interaction:
			if constexpr (scenario_type == SCENARIO_TYPE::CommonRoad || scenario_type == SCENARIO_TYPE::Lanelet2) {
				return this->lanelet_interaction_valid(vehicles_obstacle, i);
			} else {
				return true;
			}
		}

		[[nodiscard]] bool vehicle_collisions_conflict_literature(std::array<std::vector<vec2>, n_vehicles> const &vehicles_obstacles, std::array<bool, n_vehicles> &collision_indices) const {
			bool ret = true;
			for (auto i = 0; i < n_vehicles; ++i) {
				for (auto j = i + 1; j < n_vehicles; ++j) {
					if (SAT::check_collision((vec2 const *)vehicles_obstacles[i].data(), vehicles_obstacles[i].size() - 1, (vec2 const *)vehicles_obstacles[j].data(), vehicles_obstacles[j].size() - 1)) {
						collision_indices[i] = true;
						collision_indices[j] = true;
						ret = false;
					}
				}
			}

			return ret;
		}

		bool check_path(std::array<Node<1> *, n_vehicles> const &nodes, ConflictBasedNode<n_vehicles> *current_part) {
			auto const k = nodes.front()->k();

			if (k > 1) {  // because root does not need to be checked.
				std::array<Node<1> *, n_vehicles> new_nodes;

				for (auto i = 0; i < n_vehicles; ++i) {
					new_nodes[i] = nodes[i]->parent();
				}

				if (!check_path(new_nodes, current_part)) return false;
			}

			std::array<std::vector<vec2>, n_vehicles> vehicles_obstacles;
			if (k < CentralizedGraphSearch<n_vehicles, scenario_type>::_config->n_hp()) {
				vehicles_obstacles = this->_mpa->template calc_vehicles_obstacles_without_offset_conflict_array<n_vehicles>(nodes);
			} else {
				vehicles_obstacles = this->_mpa->template calc_vehicles_obstacles_large_offset_conflict_array<n_vehicles>(nodes);
			}

			std::array<bool, n_vehicles> collision_indices = {false};
			if (!vehicle_collisions_conflict_literature(vehicles_obstacles, collision_indices)) {
				for (auto i = 0; i < n_vehicles; ++i) {
					if (collision_indices[i]) current_part->_children[i] = new ConflictBasedNode<n_vehicles>(current_part, vehicles_obstacles[i], k, i);
				}
				return false;
			}

			return true;
		}

		OptimalNode<n_vehicles> *convert_nodes(std::array<OptimalNode<1> *, n_vehicles> const &mins) const {
			return convert_nodes_impl(mins, CentralizedGraphSearch<n_vehicles, scenario_type>::_config->n_hp(), std::make_integer_sequence<unsigned int, n_vehicles>{});
		}

		template <unsigned int... I>
		OptimalNode<n_vehicles> *convert_nodes_impl(std::array<OptimalNode<1> *, n_vehicles> const &mins, unsigned int const k, std::integer_sequence<unsigned int, I...> &&Indices) const {
			if (k == 0) {
				return nullptr;
			}

			return new OptimalNode<n_vehicles>(convert_nodes_impl({static_cast<OptimalNode<1> *>(mins[I]->parent())...}, k - 1, std::move(Indices)), k, {mins[I]->trim(0 + I * 0)...}, {mins[I]->x(0 + I * 0)...}, {mins[I]->y(0 + I * 0)...},
			    {mins[I]->yaw(0 + I * 0)...}, (mins[I]->g() + ...), (mins[I]->h() + ...));
		}

		void expand_node(OptimalNode<1> *const node, unsigned int const index, std::priority_queue<OptimalNode<1> *, std::vector<OptimalNode<1> *>, typename OptimalNode<1>::priority_queue_comparison> &pq) {
			std::array<std::uint8_t, 1> const sizes = CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->template get_reachable_states_sizes<1>(node->trims(), node->k());
			std::array<unsigned int, 1> const multipliers = CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->template get_reachable_states_multipliers<1>(sizes);

			auto const n_children = multipliers.back() * sizes.back();

			node->create_children(n_children);

			for (auto i = 0; i < n_children; ++i) {
				auto const next_states = CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->template reachable_states<1>(multipliers, i, node->trims(), sizes, node->k());

				auto *const new_node = new OptimalNode<1>(node, node->g(), node->k() + 1);
				this->_mpa->template init_node<1>(new_node, next_states);
				this->init_cost(new_node, index);

				node->child(i) = new_node;
				pq.push(new_node);
			}
		}
	};
}  // namespace GraphBasedPlanning