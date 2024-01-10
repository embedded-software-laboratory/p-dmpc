#pragma once
#include <Node.h>
#include <OptimalNode.h>
#include <SAT.h>

#include <array>
#include <memory>
#include <ranges>
#include <set>

#include "ConfigData.h"
#include "MPA.h"
#include "VehicleData.h"

namespace GraphBasedPlanning {
	// CentralizedGraphSearch is an interface to all inheriting graph search implementations.
	template <unsigned int n_vehicles, SCENARIO_TYPE scenario_type>
	class CentralizedGraphSearch : protected SAT {
	   protected:
		std::shared_ptr<GraphBasedPlanning::ConfigData const> const &_config;
		std::shared_ptr<GraphBasedPlanning::MPA const> const &_mpa;
		std::array<VehicleData<scenario_type>, n_vehicles> _vehicle_data;

		CentralizedGraphSearch(std::shared_ptr<GraphBasedPlanning::ConfigData const> const &config, std::shared_ptr<GraphBasedPlanning::MPA const> const &mpa, std::array<VehicleData<scenario_type>, n_vehicles> const &&vehicle_data)
		    : _config(config), _mpa(mpa), _vehicle_data(vehicle_data) {}

		[[nodiscard]] virtual bool is_target_reached(Node<n_vehicles> const *const node) const { return node->k() >= _config->n_hp(); }

		[[nodiscard]] bool is_path_valid(std::array<std::vector<vec2>, n_vehicles> const &vehicles_obstacles) const {
			// vehicle interaction:
			for (auto i = 0; i < n_vehicles; ++i) {
				for (auto j = i + 1; j < n_vehicles; ++j) {
					if (SAT::check_collision((vec2 const *)vehicles_obstacles[i].data(), vehicles_obstacles[i].size() - 1, (vec2 const *)vehicles_obstacles[j].data(), vehicles_obstacles[j].size() - 1)) return false;
				}
			}

			// lanelet interaction:
			if constexpr (scenario_type == SCENARIO_TYPE::CommonRoad) {
				for (auto i = 0; i < n_vehicles; ++i) {
					if (!lanelet_interaction_valid(vehicles_obstacles[i], i)) return false;
				}
			}
			return true;
		}

		[[nodiscard]] bool lanelet_interaction_valid(std::vector<vec2> const &vehicle_polygon, std::integral auto const vehicle_id) const requires(scenario_type != SCENARIO_TYPE::Circle) {
			double const max_x = std::max_element(vehicle_polygon.begin(), vehicle_polygon.end(), [](vec2 const a, vec2 const b) { return a.x < b.x; })->x;
			double const max_y = std::max_element(vehicle_polygon.begin(), vehicle_polygon.end(), [](vec2 const a, vec2 const b) { return a.y < b.y; })->y;
			double const min_x = std::min_element(vehicle_polygon.begin(), vehicle_polygon.end(), [](vec2 const a, vec2 const b) { return a.x < b.x; })->x;
			double const min_y = std::min_element(vehicle_polygon.begin(), vehicle_polygon.end(), [](vec2 const a, vec2 const b) { return a.y < b.y; })->y;
			for (auto j = 0; j < _vehicle_data[vehicle_id].predicted_left_lanelet_boundary().size() - 2; j += 2) {
				if ((max_x < _vehicle_data[vehicle_id].predicted_left_lanelet_boundary()[j] && max_x < _vehicle_data[vehicle_id].predicted_left_lanelet_boundary()[j + 2]) ||
				    (min_x > _vehicle_data[vehicle_id].predicted_left_lanelet_boundary()[j] && min_x > _vehicle_data[vehicle_id].predicted_left_lanelet_boundary()[j + 2]) ||
				    (max_y < _vehicle_data[vehicle_id].predicted_left_lanelet_boundary()[j + 1] && max_y < _vehicle_data[vehicle_id].predicted_left_lanelet_boundary()[j + 3]) ||
				    (min_y > _vehicle_data[vehicle_id].predicted_left_lanelet_boundary()[j + 1] && min_y > _vehicle_data[vehicle_id].predicted_left_lanelet_boundary()[j + 3])) {
					continue;
				}

				if (SAT::check_collision((vec2 const *)vehicle_polygon.data(), vehicle_polygon.size() - 1, (vec2 const *)(_vehicle_data[vehicle_id].predicted_left_lanelet_boundary().data() + j), 2U)) return false;
			}

			for (auto j = 0; j < _vehicle_data[vehicle_id].predicted_right_lanelet_boundary().size() - 2; j += 2) {
				if ((max_x < _vehicle_data[vehicle_id].predicted_right_lanelet_boundary()[j] && max_x < _vehicle_data[vehicle_id].predicted_right_lanelet_boundary()[j + 2]) ||
				    (min_x > _vehicle_data[vehicle_id].predicted_right_lanelet_boundary()[j] && min_x > _vehicle_data[vehicle_id].predicted_right_lanelet_boundary()[j + 2]) ||
				    (max_y < _vehicle_data[vehicle_id].predicted_right_lanelet_boundary()[j + 1] && max_y < _vehicle_data[vehicle_id].predicted_right_lanelet_boundary()[j + 3]) ||
				    (min_y > _vehicle_data[vehicle_id].predicted_right_lanelet_boundary()[j + 1] && min_y > _vehicle_data[vehicle_id].predicted_right_lanelet_boundary()[j + 3])) {
					continue;
				}

				if (SAT::check_collision((vec2 const *)vehicle_polygon.data(), vehicle_polygon.size() - 1, (vec2 const *)(_vehicle_data[vehicle_id].predicted_right_lanelet_boundary().data() + j), 2U)) return false;
			}
			return true;
		}

		void init_cost(Node<1> *const new_node, std::integral auto const vehicle_id) const {
			// cost to come:    Distance to reference trajectory points squared to
			// conform with
			//                  J = (x-x_ref)' Q (x-x_ref)
			double norm_x_g = new_node->x(0) - _vehicle_data[vehicle_id].reference_trajectory_point(new_node->k() - 1).x;
			double norm_y_g = new_node->y(0) - _vehicle_data[vehicle_id].reference_trajectory_point(new_node->k() - 1).y;
			double norm_squared_g = norm_x_g * norm_x_g + norm_y_g * norm_y_g;

			new_node->g() += norm_squared_g;
		}
		void init_cost(OptimalNode<1> *const new_node, std::integral auto const vehicle_id) const {
			// cost to come:    Distance to reference trajectory points squared to
			// conform with
			//                  J = (x-x_ref)' Q (x-x_ref)
			double norm_x_g = new_node->x(0) - _vehicle_data[vehicle_id].reference_trajectory_point(new_node->k() - 1).x;
			double norm_y_g = new_node->y(0) - _vehicle_data[vehicle_id].reference_trajectory_point(new_node->k() - 1).y;
			double norm_squared_g = norm_x_g * norm_x_g + norm_y_g * norm_y_g;

			new_node->g() += norm_squared_g;

			// cost to go:      same as cost to come
			//                  subtract squared distance traveled for every
			//                  timestep and vehicle
			double max_distance_traveled = 0.0;
			for (auto j = new_node->k(); j < _config->n_hp(); ++j) {
				max_distance_traveled += _config->dt() * _vehicle_data[vehicle_id].v_ref(j);

				double const norm_x_h = new_node->x(0) - _vehicle_data[vehicle_id].reference_trajectory_point(j).x;
				double const norm_y_h = new_node->y(0) - _vehicle_data[vehicle_id].reference_trajectory_point(j).y;
				double const norm_h = std::sqrt(norm_x_h * norm_x_h + norm_y_h * norm_y_h) - max_distance_traveled;
				double const norm_squared_h = norm_h * norm_h;

				new_node->h() += norm_squared_h;
			}
		}
		void init_cost(Node<n_vehicles> *const new_node) const {
			for (auto i = 0; i < n_vehicles; ++i) {
				// cost to come:    Distance to reference trajectory points squared to
				// conform with
				//                  J = (x-x_ref)' Q (x-x_ref)
				double norm_x_g = new_node->x(i) - _vehicle_data[i].reference_trajectory_point(new_node->k() - 1).x;
				double norm_y_g = new_node->y(i) - _vehicle_data[i].reference_trajectory_point(new_node->k() - 1).y;
				double norm_squared_g = norm_x_g * norm_x_g + norm_y_g * norm_y_g;

				new_node->g() += norm_squared_g;
			}
		}
		void init_cost(OptimalNode<n_vehicles> *const new_node) const {
			for (auto i = 0; i < n_vehicles; ++i) {
				// cost to come:    Distance to reference trajectory points squared to
				// conform with
				//                  J = (x-x_ref)' Q (x-x_ref)
				double norm_x_g = new_node->x(i) - _vehicle_data[i].reference_trajectory_point(new_node->k() - 1).x;
				double norm_y_g = new_node->y(i) - _vehicle_data[i].reference_trajectory_point(new_node->k() - 1).y;
				double norm_squared_g = norm_x_g * norm_x_g + norm_y_g * norm_y_g;

				new_node->g() += norm_squared_g;

				// cost to go:      same as cost to come
				//                  subtract squared distance traveled for every
				//                  timestep and vehicle
				double max_distance_traveled = 0.0;
				for (auto j = new_node->k(); j < _config->n_hp(); ++j) {
					max_distance_traveled += _config->dt() * _vehicle_data[i].v_ref(j);

					double const norm_x_h = new_node->x(i) - _vehicle_data[i].reference_trajectory_point(j).x;
					double const norm_y_h = new_node->y(i) - _vehicle_data[i].reference_trajectory_point(j).y;
					double const norm_h = std::sqrt(norm_x_h * norm_x_h + norm_y_h * norm_y_h) - max_distance_traveled;
					double const norm_squared_h = norm_h * norm_h;

					new_node->h() += norm_squared_h;
				}
			}
		}

		bool check_path(Node<n_vehicles> const *node) const {
			// without root, because vehicle is already on the position; begin at last hp, because highest chance of collision
			if (!this->is_path_valid(this->_mpa->calc_vehicles_obstacles_large_offset(node))) {
				return false;
			}

			for (int k = this->_config->n_hp() - 1; k; --k) {
				node = node->parent();

				if (!this->is_path_valid(this->_mpa->calc_vehicles_obstacles_without_offset(node))) {
					return false;
				}
			}

			return true;
		}

	   public:
		virtual void clean() = 0;
		virtual Node<n_vehicles> const *search(Node<n_vehicles> root) = 0;
	};
}  // namespace GraphBasedPlanning