#pragma once

#include <OptimalNode.h>

#include <array>
#include <numbers>
#include <queue>
#include <tuple>
#include <MatlabException.h>

#include "CentralizedGraphSearch.h"
#include "CentralizedOptimalSimple.h"
#include "ConfigData.h"
#include "MPA.h"
#include "VehicleData.h"

namespace GraphBasedPlanning {
	template <unsigned int n_vehicles, SCENARIO_TYPE scenario_type>
	class CentralizedConflictBasedFast : public CentralizedGraphSearch<n_vehicles, scenario_type> {
		Node<n_vehicles> *_min = nullptr;
		std::array<std::priority_queue<OptimalNode<1> *, std::vector<OptimalNode<1> *>, typename OptimalNode<1>::priority_queue_comparison>, n_vehicles> _pqs;

	   public:
		CentralizedConflictBasedFast(std::shared_ptr<GraphBasedPlanning::ConfigData const> const &config, std::shared_ptr<GraphBasedPlanning::MPA const> const &mpa, std::array<VehicleData<scenario_type>, n_vehicles> const &&vehicle_data)
		    : CentralizedGraphSearch<n_vehicles, scenario_type>(config, mpa, std::forward<decltype(vehicle_data)>(vehicle_data)) {}
		Node<n_vehicles> const *search(Node<n_vehicles> const root) final {
			auto produce_roots = [root]<std::size_t... I>(std::index_sequence<I...>) -> std::array<OptimalNode<1>, n_vehicles> { return {OptimalNode<1>({root.trim(I)}, {root.x(I)}, {root.y(I)}, {root.yaw(I)})...}; };

			std::array<Node<1> *, n_vehicles> mins = {nullptr};
			std::array<std::vector<std::vector<std::vector<vec2>>>, n_vehicles> collisions_before;

			for (auto &e : collisions_before) {
				e.resize(this->_config->n_hp());
			}

			do {
				std::array<OptimalNode<1>, n_vehicles> roots = produce_roots(std::make_index_sequence<n_vehicles>{});
				for (auto i = 0; i < n_vehicles; ++i) {
					expand_node(&roots[i], i);

					while (true) {
						if (_pqs[i].empty()) throw MatlabException("not feasible!");

						OptimalNode<1> *const current_node = _pqs[i].top();
						_pqs[i].pop();

						std::vector<vec2> vehicles_obstacle;
						if (current_node->k() < this->_config->n_hp()) {
							vehicles_obstacle = this->_mpa->calc_vehicles_obstacles_without_offset_conflict(current_node);
						} else {
							vehicles_obstacle = this->_mpa->calc_vehicles_obstacles_large_offset_conflict(current_node);
						}

						if (!this->is_path_valid(vehicles_obstacle, i, current_node->k(), collisions_before)) {
							continue;
						}

						if (current_node->k() >= this->_config->n_hp()) [[unlikely]] {
							mins[i] = current_node;
							break;
						}

						expand_node(current_node, i);
					}
				}
				if (check_path_new(mins, collisions_before)) {
					save(current_used_memory);
					return convert_nodes(mins);
				}

				for (auto &e : _pqs) e = std::priority_queue<OptimalNode<1> *, std::vector<OptimalNode<1> *>, typename OptimalNode<1>::priority_queue_comparison>();
			} while (true);
		}
		void clean() final {
			delete this;  // commit suicide
		}

		~CentralizedConflictBasedFast() {
			delete _min;
		}

	   private:
		bool check_path_new(std::array<Node<1> *, n_vehicles> const &nodes, std::array<std::vector<std::vector<std::vector<vec2>>>, n_vehicles> &collisions_before) {
			auto const k = nodes.front()->k();

			if (k > 1) {  // because root does not need to be checked.
				std::array<Node<1> *, n_vehicles> new_nodes;

				for (auto i = 0; i < n_vehicles; ++i) {
					new_nodes[i] = nodes[i]->parent();
				}

				if (!check_path_new(new_nodes, collisions_before)) return false;
			}

			std::array<std::vector<vec2>, n_vehicles> vehicles_obstacles;
			if (k < this->_config->n_hp()) {
				vehicles_obstacles = this->_mpa->template calc_vehicles_obstacles_without_offset_conflict_array<n_vehicles>(nodes);
			} else {
				vehicles_obstacles = this->_mpa->template calc_vehicles_obstacles_large_offset_conflict_array<n_vehicles>(nodes);
			}

			int i, j;
			if (!this->vehicle_collisions_conflict2(vehicles_obstacles, i, j)) {
				vec2 boundary1 = perpendicular({nodes[i]->x(0) - nodes[j]->x(0), nodes[i]->y(0) - nodes[j]->y(0)});
				vec2 yaw1 = {std::cos(nodes[i]->yaw(0)), std::sin(nodes[i]->yaw(0))};
				auto dot1 = yaw1.x * boundary1.x + yaw1.y * boundary1.y;
				auto det1 = yaw1.x * boundary1.y - yaw1.y * boundary1.x;
				auto angle1 = atan2(det1, dot1);

				vec2 boundary2 = perpendicular({nodes[j]->x(0) - nodes[i]->x(0), nodes[j]->y(0) - nodes[i]->y(0)});
				vec2 yaw2 = {std::cos(nodes[j]->yaw(0)), std::sin(nodes[j]->yaw(0))};
				auto dot2 = yaw2.x * boundary2.x + yaw2.y * boundary2.y;
				auto det2 = yaw2.x * boundary2.y - yaw2.y * boundary2.x;
				auto angle2 = atan2(det2, dot2);

				if (angle1 > 0.0 && angle2 < 0.0) {
					collisions_before[j][k - 1].push_back(vehicles_obstacles[j]);
					// Printer::println(j + 1, " is before ", i + 1);
				} else if (angle1 < 0.0 && angle2 > 0.0) {
					collisions_before[i][k - 1].push_back(vehicles_obstacles[i]);
					// Printer::println(i + 1, " is before ", j + 1);
				} else {
					collisions_before[i][k - 1].push_back(vehicles_obstacles[i]);
					// Printer::println(i + 1, " approaches ", j + 1);
				}

				return false;
			}

			return true;
		}

		bool check_path(Node<n_vehicles> const *node) const {
			if (!this->is_path_valid(this->_mpa->calc_vehicles_obstacles_large_offset(node))) {
				return false;
			}

			for (auto i = this->_config->n_hp() - 2; i; --i) {  // without root, because vehicle is already on the position; begin at last hp, because highest chance of collision
				node = node->parent();

				if (!this->is_path_valid(this->_mpa->calc_vehicles_obstacles_without_offset(node))) {
					return false;
				}
			}

			return true;
		}

		Node<n_vehicles> *convert_nodes(std::array<Node<1> *, n_vehicles> const mins) {
			return convert_nodes_impl(mins, this->_config->n_hp(), std::make_integer_sequence<unsigned int, n_vehicles>{});
		}

		template <unsigned int... I>
		Node<n_vehicles> *convert_nodes_impl(std::array<Node<1> *, n_vehicles> const mins, unsigned int const k, std::integer_sequence<unsigned int, I...> &&Indices) {
			if (k == 0) {
				return nullptr;
			}

			return new Node<n_vehicles>(
			    convert_nodes_impl({mins[I]->parent()...}, k - 1, std::move(Indices)), k, {mins[I]->trim(0 + I * 0)...}, {mins[I]->x(0 + I * 0)...}, {mins[I]->y(0 + I * 0)...}, {mins[I]->yaw(0 + I * 0)...}, (mins[I]->g() + ...));
		}

		void expand_node(OptimalNode<1> *const node, unsigned int const index) {
			std::array<std::uint8_t, 1> const sizes = this->_mpa->template get_reachable_states_sizes<1>(node->trims(), node->k());
			std::array<unsigned int, 1> const multipliers = this->_mpa->template get_reachable_states_multipliers<1>(sizes);

			auto const n_children = multipliers.back() * sizes.back();

			node->create_children(n_children);

			for (auto i = 0; i < n_children; ++i) {
				auto const next_states = this->_mpa->template reachable_states<1>(multipliers, i, node->trims(), sizes, node->k());

				auto *const new_node = new OptimalNode<1>(node, node->g(), node->k() + 1);
				this->_mpa->template init_node<1>(new_node, next_states);
				this->init_cost(new_node, index);

				node->child(i) = new_node;
				_pqs[index].push(new_node);
			}
		}
	};
}  // namespace GraphBasedPlanning