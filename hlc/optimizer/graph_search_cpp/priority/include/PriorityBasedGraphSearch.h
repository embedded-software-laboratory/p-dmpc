#pragma once
#include <PriorityBasedNode.h>
#include <SAT.h>

#include <array>
#include <memory>
#include <ranges>
#include <set>

#include "ConfigData.h"
#include "MPA.h"
#include "VehicleData.h"
#include "VehicleObstaclesData.h"

namespace GraphBasedPlanning {
	template <SCENARIO_TYPE scenario_type>
	class PriorityBasedGraphSearch : protected SAT {
	   protected:
		std::shared_ptr<GraphBasedPlanning::ConfigData const> const &_config;
		std::shared_ptr<GraphBasedPlanning::MPA const> const &_mpa;
		std::array<VehicleData<scenario_type>, 1> _vehicle_data;
		std::vector<std::vector<ColMajorMatrixAccessor<double>>> _vehicle_obstacles;

		PriorityBasedGraphSearch(std::shared_ptr<GraphBasedPlanning::ConfigData const> const &config, std::shared_ptr<GraphBasedPlanning::MPA const> const &mpa, std::array<VehicleData<scenario_type>, 1> const &&vehicle_data,
		    VehicleObstaclesData const &&vehicle_obstacles_data)
		    : _config(config), _mpa(mpa), _vehicle_data(vehicle_data), _vehicle_obstacles(vehicle_obstacles_data) {}

		[[nodiscard]] virtual bool is_target_reached(PriorityBasedNode const *const node) const { return node->k() >= _config->n_hp(); }

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

		void init_cost(PriorityBasedNode *const new_node) const {
			// cost to come:    Distance to reference trajectory points squared to
			// conform with
			//                  J = (x-x_ref)' Q (x-x_ref)
			double norm_x_g = new_node->x(0) - _vehicle_data[0].reference_trajectory_point(new_node->k() - 1).x;
			double norm_y_g = new_node->y(0) - _vehicle_data[0].reference_trajectory_point(new_node->k() - 1).y;
			double norm_squared_g = norm_x_g * norm_x_g + norm_y_g * norm_y_g;

			new_node->g() += norm_squared_g;

			// cost to go:      same as cost to come
			//                  subtract squared distance traveled for every
			//                  timestep and vehicle
			double max_distance_traveled = 0.0;
			for (auto j = new_node->k(); j < _config->n_hp(); ++j) {
				max_distance_traveled += _config->dt_seconds() * _vehicle_data[0].v_ref(j);

				double const norm_x_h = new_node->x(0) - _vehicle_data[0].reference_trajectory_point(j).x;
				double const norm_y_h = new_node->y(0) - _vehicle_data[0].reference_trajectory_point(j).y;
				double const norm_h = std::sqrt(norm_x_h * norm_x_h + norm_y_h * norm_y_h) - max_distance_traveled;
				double const norm_squared_h = norm_h * norm_h;

				new_node->h() += norm_squared_h;
			}
		}

		void set_predicted_areas(PriorityBasedNode *const node) const {
			auto c = std::cos(node->parent()->yaw(0));
			auto s = std::sin(node->parent()->yaw(0));
			auto const &points = _mpa->area(node->parent()->trim(0), node->trim(0));
			std::vector<double> predicted_areas;
			predicted_areas.resize(points.size());

			for (unsigned int j = 0; j < predicted_areas.size(); j = j + 2) {
				predicted_areas[j] = c * points[j] - s * points[j + 1] + node->parent()->x(0);
				predicted_areas[j + 1] = s * points[j] + c * points[j + 1] + node->parent()->y(0);
			}
			node->shapes() = predicted_areas;
		}

		[[nodiscard]] bool check_vehicle_obstacles(PriorityBasedNode *const node) const {
			uint16_t n_obstacles = _vehicle_obstacles[node->k() - 1].size();
			for (unsigned int i = 0; i < n_obstacles; ++i) {
				if (SAT::check_collision(
				        reinterpret_cast<vec2 const *>(node->shapes().data()), node->shapes().size() / 2 - 1, reinterpret_cast<vec2 const *>(_vehicle_obstacles[node->k() - 1][i].data()), _vehicle_obstacles[node->k() - 1][i].size() / 2 - 1)) {
					return false;
				}
			}
			return true;
		}

	   public:
		virtual void clean() = 0;
		virtual PriorityBasedNode const *search(PriorityBasedNode const root) = 0;
		virtual std::uint64_t get_n_expanded() = 0;
	};
}  // namespace GraphBasedPlanning