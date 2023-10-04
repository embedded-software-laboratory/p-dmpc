#pragma once

#include <Node.h>
#include <vec2.h>

#include <algorithm>
#include <cstdint>
#include <vector>

#include "Maneuver.h"

namespace GraphBasedPlanning {
	class MPA {
		std::vector<std::vector<std::vector<std::uint8_t>>> const _reachability_list;
		std::vector<std::vector<std::vector<std::uint8_t>>> const _transition_matrix_single;
		std::vector<std::vector<double>> const _transition_matrix_mean_speed;
		std::vector<std::vector<Maneuver<double>>> const _maneuvers;
		double _max_speed;
		std::vector<std::uint16_t> _distance_to_equilibrium;

	   public:
		MPA(std::vector<std::vector<std::vector<std::uint8_t>>> reachability_list, std::vector<std::vector<std::vector<std::uint8_t>>> transition_matrix_single, std::vector<std::vector<double>> const&& transition_matrix_mean_speed,
		    std::vector<std::vector<Maneuver<double>>> const&& maneuvers, std::vector<std::uint16_t> const&& distance_to_equilibrium);

		std::vector<double> const& area_large_offset(std::integral auto const from, std::integral auto const to) const { return _maneuvers[from][to].area_large_offset; }
		std::vector<double> const& area(std::integral auto const from, std::integral auto const to) const { return _maneuvers[from][to].area; }
		std::vector<double> const& area_without_offset(std::integral auto const from, std::integral auto const to) const { return _maneuvers[from][to].area_without_offset; }
		[[nodiscard]] double dx(std::integral auto const from, std::integral auto const to) const { return _maneuvers[from][to].dx; }
		[[nodiscard]] double dy(std::integral auto const from, std::integral auto const to) const { return _maneuvers[from][to].dy; }
		[[nodiscard]] double dyaw(std::integral auto const from, std::integral auto const to) const { return _maneuvers[from][to].dyaw; }
		std::vector<double> const& xs(std::integral auto const from, std::integral auto const to) const { return _maneuvers[from][to].xs; }
		std::vector<double> const& ys(std::integral auto const from, std::integral auto const to) const { return _maneuvers[from][to].ys; }
		std::vector<double> const& yaws(std::integral auto const from, std::integral auto const to) const { return _maneuvers[from][to].yaws; }

		std::uint16_t distance_to_equilibrium(std::integral auto const trim) const { return _distance_to_equilibrium[trim]; }
		[[nodiscard]] double mean_speed(std::integral auto const from, std::integral auto const to) const { return _transition_matrix_mean_speed[from][to]; }
		[[nodiscard]] double max_speed() const { return _max_speed; }

		std::vector<std::uint8_t> const& reachability_list(std::integral auto const k, std::integral auto const trim) const { return _reachability_list[k][trim]; }
		std::vector<std::uint8_t> const& reachability_list(std::integral auto const trim) const { return _reachability_list[0][trim]; }

		template <unsigned int n_vehicles>
		std::array<std::uint8_t, n_vehicles> get_reachable_states_sizes(std::array<std::uint8_t, n_vehicles> const& current_trims, std::integral auto const current_step) const {
			std::array<std::uint8_t, n_vehicles> sizes;

			for (auto i = 0; i < n_vehicles; ++i) {
				sizes[i] = _reachability_list[current_step][current_trims[i]].size();
			}

			return sizes;
		}

		template <unsigned int n_vehicles>
		std::array<unsigned int, n_vehicles> get_reachable_states_multipliers(std::array<std::uint8_t, n_vehicles> const& sizes) const {
			std::array<unsigned int, n_vehicles> multipliers;

			multipliers[0] = 1U;
			for (auto i = 1; i < n_vehicles; ++i) {
				multipliers[i] = multipliers[i - 1] * sizes[i - 1];
			}

			return multipliers;
		}

		template <unsigned int n_vehicles>
		std::array<std::uint8_t, n_vehicles> reachable_states(
		    std::array<unsigned int, n_vehicles> const& multipliers, unsigned int const i, std::array<std::uint8_t, n_vehicles> const& current_trims, std::array<std::uint8_t, n_vehicles> const& sizes, std::integral auto const step) const {
			std::array<std::uint8_t, n_vehicles> next_trims;

			for (auto j = 0; j < n_vehicles; ++j) {
				next_trims[j] = _reachability_list[step][current_trims[j]][i / multipliers[j] % sizes[j]];
			}

			return next_trims;
		}

		template <unsigned int n_vehicles>
		void init_node(Node<n_vehicles>* const new_node, std::array<std::uint8_t, n_vehicles> const& next_trims) const {
			for (auto i = 0; i < n_vehicles; ++i) {
				new_node->trim(i) = next_trims[i];

				auto c = std::cos(new_node->parent()->yaw(i));
				auto s = std::sin(new_node->parent()->yaw(i));

				auto const dx_ = dx(new_node->parent()->trim(i), new_node->trim(i));
				auto const dy_ = dy(new_node->parent()->trim(i), new_node->trim(i));
				auto const dyaw_ = dyaw(new_node->parent()->trim(i), new_node->trim(i));

				new_node->x(i) = c * dx_ - s * dy_ + new_node->parent()->x(i);
				new_node->y(i) = s * dx_ + c * dy_ + new_node->parent()->y(i);
				new_node->yaw(i) = dyaw_ + new_node->parent()->yaw(i);
			}
		}

		template <unsigned int n_vehicles>
		std::array<std::vector<vec2>, n_vehicles> calc_vehicles_obstacles_large_offset(Node<n_vehicles> const* const node) const {
			std::array<std::vector<vec2>, n_vehicles> vehicles_obstacles;

			for (unsigned int i = 0; i < n_vehicles; ++i) {
				auto c = std::cos(node->parent()->yaw(i));
				auto s = std::sin(node->parent()->yaw(i));

				auto const& points = area_large_offset(node->parent()->trim(i), node->trim(i));
				vehicles_obstacles[i].resize(points.size() / 2);

				for (unsigned int j = 0; j < vehicles_obstacles[i].size(); ++j) {
					vehicles_obstacles[i][j].x = c * points[j * 2] - s * points[j * 2 + 1] + node->parent()->x(i);
					vehicles_obstacles[i][j].y = s * points[j * 2] + c * points[j * 2 + 1] + node->parent()->y(i);
				}
			}

			return vehicles_obstacles;
		}

		template <unsigned int n_vehicles>
		std::array<std::vector<vec2>, n_vehicles> calc_vehicles_obstacles_without_offset(Node<n_vehicles> const* const node) const {
			std::array<std::vector<vec2>, n_vehicles> vehicles_obstacles;

			for (unsigned int i = 0; i < n_vehicles; ++i) {
				auto c = std::cos(node->parent()->yaw(i));
				auto s = std::sin(node->parent()->yaw(i));

				auto const& points = area_without_offset(node->parent()->trim(i), node->trim(i));
				vehicles_obstacles[i].resize(points.size() / 2);

				for (unsigned int j = 0; j < vehicles_obstacles[i].size(); ++j) {
					vehicles_obstacles[i][j].x = c * points[j * 2] - s * points[j * 2 + 1] + node->parent()->x(i);
					vehicles_obstacles[i][j].y = s * points[j * 2] + c * points[j * 2 + 1] + node->parent()->y(i);
				}
			}

			return vehicles_obstacles;
		}

		template <unsigned int n_vehicles>
		std::array<std::vector<vec2>, n_vehicles> calc_vehicles_obstacles_without_offset_conflict_array(std::array<Node<1>*, n_vehicles> const& nodes) const {
			std::array<std::vector<vec2>, n_vehicles> vehicles_obstacles;

			for (auto i = 0; i < n_vehicles; ++i) {
				auto c = std::cos(nodes[i]->parent()->yaw(0));
				auto s = std::sin(nodes[i]->parent()->yaw(0));

				auto const& points = area_without_offset(nodes[i]->parent()->trim(0), nodes[i]->trim(0));
				vehicles_obstacles[i].resize(points.size() / 2);

				for (auto j = 0; j < vehicles_obstacles[i].size(); ++j) {
					vehicles_obstacles[i][j].x = c * points[j * 2] - s * points[j * 2 + 1] + nodes[i]->parent()->x(0);
					vehicles_obstacles[i][j].y = s * points[j * 2] + c * points[j * 2 + 1] + nodes[i]->parent()->y(0);
				}
			}

			return vehicles_obstacles;
		}

		template <unsigned int n_vehicles>
		std::array<std::vector<vec2>, n_vehicles> calc_vehicles_obstacles_large_offset_conflict_array(std::array<Node<1>*, n_vehicles> const& nodes) const {
			std::array<std::vector<vec2>, n_vehicles> vehicles_obstacles;

			for (auto i = 0; i < n_vehicles; ++i) {
				auto c = std::cos(nodes[i]->parent()->yaw(0));
				auto s = std::sin(nodes[i]->parent()->yaw(0));

				auto const& points = area_large_offset(nodes[i]->parent()->trim(0), nodes[i]->trim(0));
				vehicles_obstacles[i].resize(points.size() / 2);

				for (auto j = 0; j < vehicles_obstacles[i].size(); ++j) {
					vehicles_obstacles[i][j].x = c * points[j * 2] - s * points[j * 2 + 1] + nodes[i]->parent()->x(0);
					vehicles_obstacles[i][j].y = s * points[j * 2] + c * points[j * 2 + 1] + nodes[i]->parent()->y(0);
				}
			}

			return vehicles_obstacles;
		}

		std::vector<vec2> calc_vehicles_obstacles_without_offset_conflict(Node<1> const* const node) const {
			std::vector<vec2> vehicles_obstacle;

			auto c = std::cos(node->parent()->yaw(0));
			auto s = std::sin(node->parent()->yaw(0));

			auto const& points = area_without_offset(node->parent()->trim(0), node->trim(0));
			vehicles_obstacle.resize(points.size() / 2);

			for (auto j = 0; j < vehicles_obstacle.size(); ++j) {
				vehicles_obstacle[j].x = c * points[j * 2] - s * points[j * 2 + 1] + node->parent()->x(0);
				vehicles_obstacle[j].y = s * points[j * 2] + c * points[j * 2 + 1] + node->parent()->y(0);
			}

			return vehicles_obstacle;
		}

		std::vector<vec2> calc_vehicles_obstacles_large_offset_conflict(Node<1> const* const node) const {
			std::vector<vec2> vehicles_obstacle;

			auto c = std::cos(node->parent()->yaw(0));
			auto s = std::sin(node->parent()->yaw(0));

			auto const& points = area_large_offset(node->parent()->trim(0), node->trim(0));
			vehicles_obstacle.resize(points.size() / 2);

			for (auto j = 0; j < vehicles_obstacle.size(); ++j) {
				vehicles_obstacle[j].x = c * points[j * 2] - s * points[j * 2 + 1] + node->parent()->x(0);
				vehicles_obstacle[j].y = s * points[j * 2] + c * points[j * 2 + 1] + node->parent()->y(0);
			}

			return vehicles_obstacle;
		}
	};
}  // namespace GraphBasedPlanning