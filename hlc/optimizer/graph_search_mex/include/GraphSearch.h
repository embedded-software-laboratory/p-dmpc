#pragma once

#include <boost/geometry/geometry.hpp>
#include <cstdint>
#include <functional>
#include <random>
#include <vector>
#include <algorithm>

#include "ColMajorAccessor.h"
#include "CollisionDetection.h"
#include "ConfigData.h"
#include "MPA.h"
#include "Node.h"

namespace GraphBasedPlanning {
	class GraphSearch : protected virtual MPA {
	   private:
		[[nodiscard]] inline virtual unsigned int &n_trims() = 0;
		[[nodiscard]] inline virtual unsigned int n_trims() const = 0;

		[[nodiscard]] inline virtual unsigned int &n_hp() = 0;
		[[nodiscard]] inline virtual unsigned int n_hp() const = 0;

		[[nodiscard]] inline virtual unsigned int &n_vehicles() = 0;
		[[nodiscard]] inline virtual unsigned int n_vehicles() const = 0;

		[[nodiscard]] inline virtual unsigned int &tick_per_step() = 0;
		[[nodiscard]] inline virtual unsigned int tick_per_step() const = 0;

		[[nodiscard]] inline virtual double &dt() = 0;
		[[nodiscard]] inline virtual double dt() const = 0;

		[[nodiscard]] inline virtual SCENARIO_TYPE &scenario_type() = 0;
		[[nodiscard]] inline virtual SCENARIO_TYPE scenario_type() const = 0;

		[[nodiscard]] inline virtual bool &is_pb() = 0;
		[[nodiscard]] inline virtual bool is_pb() const = 0;

		[[nodiscard]] inline virtual std::vector<std::vector<CollisionDetection::vec2>> &reference_trajectory() = 0;
		[[nodiscard]] inline virtual std::vector<std::vector<CollisionDetection::vec2>> const &reference_trajectory() const = 0;

	   protected:
		ColMajorTensorAccessor<FLOATING_POINT_TYPE> _reference_trajectory_points;
		ColMajorMatrixAccessor<FLOATING_POINT_TYPE> _v_ref;
		std::vector<std::array<ColMajorMatrixAccessor<FLOATING_POINT_TYPE>, 2>> _predicted_lanelet_boundary;
		std::vector<std::vector<ColMajorMatrixAccessor<FLOATING_POINT_TYPE>>> _vehicle_obstacles;

	   protected:
		std::tuple<std::vector<ColMajorMatrixAccessor<FLOATING_POINT_TYPE>>, ColMajorMatrixAccessor<std::uint8_t>, std::vector<ColMajorMatrixAccessor<FLOATING_POINT_TYPE>>, bool> respond_matlab_centralized(
		    Node<FLOATING_POINT_TYPE> const *solution) {
			std::vector<ColMajorMatrixAccessor<FLOATING_POINT_TYPE>> next_nodes(n_hp(), ColMajorMatrixAccessor<FLOATING_POINT_TYPE>({n_vehicles(), 8U}));
			ColMajorMatrixAccessor<std::uint8_t> predicted_trims({n_vehicles(), n_hp() + 1});
			std::vector<ColMajorMatrixAccessor<FLOATING_POINT_TYPE>> y_predicted(n_vehicles());  //, ColMajorMatrixAccessor<FLOATING_POINT_TYPE>({(n_hp + 1) * MPA::n_plot_points(), 4U}));
			std::vector<std::vector<std::uint8_t>> y_predicted_trims(n_vehicles());
			std::vector<std::vector<FLOATING_POINT_TYPE>> y_predicted_yaws(n_vehicles());
			std::vector<std::vector<FLOATING_POINT_TYPE>> y_predicted_ys(n_vehicles());
			std::vector<std::vector<FLOATING_POINT_TYPE>> y_predicted_xs(n_vehicles());
			bool is_exhausted = false;

			if (solution == nullptr) {
				is_exhausted = true;
				return {next_nodes, predicted_trims, y_predicted, is_exhausted};
			}

			while (true) {  // get the successor node of root of the optimal path
				for (unsigned int i = 0; i < n_vehicles(); ++i) {
					predicted_trims(i, solution->k()) = solution->trim(i) + 1U;  // convert to matlab trims

					y_predicted_trims[i].insert(y_predicted_trims[i].begin(), tick_per_step() + 1, solution->parent()->trim(i) + 1U);  // convert to matlab trims

					{
						std::vector<FLOATING_POINT_TYPE> y_predicted_yaws_tmp = yaws(solution->parent()->trim(i), solution->trim(i));
						for (auto &e : y_predicted_yaws_tmp) {
							e += solution->parent()->yaw(i);
						}
						y_predicted_yaws[i].insert(y_predicted_yaws[i].begin(), y_predicted_yaws_tmp.begin(), y_predicted_yaws_tmp.end());
					}

					{
						auto c = std::cos(solution->parent()->yaw(i));
						auto s = std::sin(solution->parent()->yaw(i));

						std::vector<FLOATING_POINT_TYPE> y_predicted_ys_tmp = ys(solution->parent()->trim(i), solution->trim(i));
						std::vector<FLOATING_POINT_TYPE> y_predicted_xs_tmp = xs(solution->parent()->trim(i), solution->trim(i));
						for (unsigned int j = 0; j < y_predicted_ys_tmp.size(); ++j) {
							y_predicted_ys_tmp[j] = s * y_predicted_xs_tmp[j] + c * y_predicted_ys_tmp[j] + solution->parent()->y(i);
						}
						y_predicted_ys[i].insert(y_predicted_ys[i].begin(), y_predicted_ys_tmp.begin(), y_predicted_ys_tmp.end());

						y_predicted_ys_tmp = ys(solution->parent()->trim(i), solution->trim(i));
						for (unsigned int j = 0; j < y_predicted_ys_tmp.size(); ++j) {
							y_predicted_xs_tmp[j] = c * y_predicted_xs_tmp[j] - s * y_predicted_ys_tmp[j] + solution->parent()->x(i);
						}
						y_predicted_xs[i].insert(y_predicted_xs[i].begin(), y_predicted_xs_tmp.begin(), y_predicted_xs_tmp.end());
					}
					// values see NodeInfo.m
					next_nodes[solution->k() - 1](i, 0) = solution->x(i);
					next_nodes[solution->k() - 1](i, 1) = solution->y(i);
					next_nodes[solution->k() - 1](i, 2) = solution->yaw(i);
					next_nodes[solution->k() - 1](i, 3) = solution->trim(i) + 1.0;  // convert to matlab trims
					next_nodes[solution->k() - 1](i, 4) = solution->g();
					next_nodes[solution->k() - 1](i, 5) = solution->h();
					next_nodes[solution->k() - 1](i, 6) = solution->k();
				}
				if (solution->k() == 1) break;
				solution = solution->parent();
			}

			for (unsigned int i = 0; i < n_vehicles(); ++i) {

				predicted_trims(i, solution->parent()->k()) = solution->parent()->trim(i) + 1U;

				y_predicted_xs[i].insert(y_predicted_xs[i].end(), y_predicted_ys[i].begin(), y_predicted_ys[i].end());
				y_predicted_xs[i].insert(y_predicted_xs[i].end(), y_predicted_yaws[i].begin(), y_predicted_yaws[i].end());
				y_predicted_xs[i].insert(y_predicted_xs[i].end(), y_predicted_trims[i].begin(), y_predicted_trims[i].end());

				y_predicted[i] = ColMajorMatrixAccessor(std::move(y_predicted_xs[i]), {(tick_per_step() + 1U) * n_hp(), 4U});
			}

			return {next_nodes, predicted_trims, y_predicted, is_exhausted};
		}

		std::tuple<std::vector<ColMajorMatrixAccessor<FLOATING_POINT_TYPE>>, ColMajorMatrixAccessor<std::uint8_t>, std::vector<ColMajorMatrixAccessor<FLOATING_POINT_TYPE>>, std::vector<ColMajorMatrixAccessor<FLOATING_POINT_TYPE>>, bool>
		respond_matlab_pb(Node<FLOATING_POINT_TYPE> const *solution) {
			std::vector<ColMajorMatrixAccessor<FLOATING_POINT_TYPE>> next_nodes(n_hp(), ColMajorMatrixAccessor<FLOATING_POINT_TYPE>({n_vehicles(), 8U}));
			ColMajorMatrixAccessor<std::uint8_t> predicted_trims({n_vehicles(), n_hp() + 1});
			std::vector<ColMajorMatrixAccessor<FLOATING_POINT_TYPE>> y_predicted(n_vehicles());  //, ColMajorMatrixAccessor<FLOATING_POINT_TYPE>({(n_hp + 1) * MPA::n_plot_points(), 4U}));
			std::vector<std::vector<std::uint8_t>> y_predicted_trims(n_vehicles());
			std::vector<std::vector<FLOATING_POINT_TYPE>> y_predicted_yaws(n_vehicles());
			std::vector<std::vector<FLOATING_POINT_TYPE>> y_predicted_ys(n_vehicles());
			std::vector<std::vector<FLOATING_POINT_TYPE>> y_predicted_xs(n_vehicles());
			std::vector<ColMajorMatrixAccessor<FLOATING_POINT_TYPE>> shapes(n_hp());
			bool is_exhausted = false;

			if (solution == nullptr) {
				is_exhausted = true;
				return {next_nodes, predicted_trims, y_predicted, shapes, is_exhausted};
			}

			while (true) {  // get the successor node of root of the optimal path
				for (unsigned int i = 0; i < n_vehicles(); ++i) {
					predicted_trims(i, solution->k()) = solution->trim(i) + 1U;  // convert to matlab trims

					y_predicted_trims[i].insert(y_predicted_trims[i].begin(), tick_per_step() + 1, solution->parent()->trim(i) + 1U);  // convert to matlab trims

					{
						std::vector<FLOATING_POINT_TYPE> y_predicted_yaws_tmp = yaws(solution->parent()->trim(i), solution->trim(i));
						for (auto &e : y_predicted_yaws_tmp) {
							e += solution->parent()->yaw(i);
						}
						y_predicted_yaws[i].insert(y_predicted_yaws[i].begin(), y_predicted_yaws_tmp.begin(), y_predicted_yaws_tmp.end());
					}

					{
						auto c = std::cos(solution->parent()->yaw(i));
						auto s = std::sin(solution->parent()->yaw(i));

						std::vector<FLOATING_POINT_TYPE> y_predicted_ys_tmp = ys(solution->parent()->trim(i), solution->trim(i));
						std::vector<FLOATING_POINT_TYPE> y_predicted_xs_tmp = xs(solution->parent()->trim(i), solution->trim(i));
						for (unsigned int j = 0; j < y_predicted_ys_tmp.size(); ++j) {
							y_predicted_ys_tmp[j] = s * y_predicted_xs_tmp[j] + c * y_predicted_ys_tmp[j] + solution->parent()->y(i);
						}
						y_predicted_ys[i].insert(y_predicted_ys[i].begin(), y_predicted_ys_tmp.begin(), y_predicted_ys_tmp.end());

						y_predicted_ys_tmp = ys(solution->parent()->trim(i), solution->trim(i));
						for (unsigned int j = 0; j < y_predicted_ys_tmp.size(); ++j) {
							y_predicted_xs_tmp[j] = c * y_predicted_xs_tmp[j] - s * y_predicted_ys_tmp[j] + solution->parent()->x(i);
						}
						y_predicted_xs[i].insert(y_predicted_xs[i].begin(), y_predicted_xs_tmp.begin(), y_predicted_xs_tmp.end());
					}
					// values see NodeInfo.m
					next_nodes[solution->k() - 1](i, 0) = solution->x(i);
					next_nodes[solution->k() - 1](i, 1) = solution->y(i);
					next_nodes[solution->k() - 1](i, 2) = solution->yaw(i);
					next_nodes[solution->k() - 1](i, 3) = solution->trim(i) + 1.0;  // convert to matlab trims
					next_nodes[solution->k() - 1](i, 4) = solution->g();
					next_nodes[solution->k() - 1](i, 5) = solution->h();
					next_nodes[solution->k() - 1](i, 6) = solution->k();

					// shapes for pb scenarios
					uint8_t k = solution->k() - 1;
					unsigned size = solution->shapes()->size();
					shapes[k] = ColMajorMatrixAccessor(std::move(*(solution->shapes())), {2U, size / 2});
				}
				if (solution->k() == 1) break;
				solution = solution->parent();
			}

			for (unsigned int i = 0; i < n_vehicles(); ++i) {
				predicted_trims(i, solution->parent()->k()) = solution->parent()->trim(i) + 1U;

				y_predicted_xs[i].insert(y_predicted_xs[i].end(), y_predicted_ys[i].begin(), y_predicted_ys[i].end());
				y_predicted_xs[i].insert(y_predicted_xs[i].end(), y_predicted_yaws[i].begin(), y_predicted_yaws[i].end());
				y_predicted_xs[i].insert(y_predicted_xs[i].end(), y_predicted_trims[i].begin(), y_predicted_trims[i].end());

				y_predicted[i] = ColMajorMatrixAccessor(std::move(y_predicted_xs[i]), {(tick_per_step() + 1U) * n_hp(), 4U});
			}

			return {next_nodes, predicted_trims, y_predicted, shapes, is_exhausted};
		}

		// check if node is last node in prediction horizon
		[[nodiscard]] bool is_target_reached(Node<FLOATING_POINT_TYPE> const *const node) const { return node->k() >= n_hp(); }

		void get_vehicle_shapes(Node<FLOATING_POINT_TYPE> const *const node, std::vector<std::vector<CollisionDetection::vec2>> &vehicles_obstacles) const {
			for (unsigned int i = 0; i < n_vehicles(); ++i) {
				auto c = std::cos(node->parent()->yaw(i));
				auto s = std::sin(node->parent()->yaw(i));

				if (node->k() < n_hp()) {
					auto const &points = area_without_offset(node->parent()->trim(i), node->trim(i));
					vehicles_obstacles[i].resize(points.size() / 2);

					for (unsigned int j = 0; j < vehicles_obstacles[i].size(); ++j) {
						vehicles_obstacles[i][j].x = c * points[j * 2] - s * points[j * 2 + 1] + node->parent()->x(i);
						vehicles_obstacles[i][j].y = s * points[j * 2] + c * points[j * 2 + 1] + node->parent()->y(i);
					}
				} else {
					auto const &points = area_large_offset(node->parent()->trim(i), node->trim(i));
					vehicles_obstacles[i].resize(points.size() / 2);

					for (unsigned int j = 0; j < vehicles_obstacles[i].size(); ++j) {
						vehicles_obstacles[i][j].x = c * points[j * 2] - s * points[j * 2 + 1] + node->parent()->x(i);
						vehicles_obstacles[i][j].y = s * points[j * 2] + c * points[j * 2 + 1] + node->parent()->y(i);
					}
				}
			}
		}

		void set_predicted_areas(Node<FLOATING_POINT_TYPE> *const node) const {
			auto c = std::cos(node->parent()->yaw(0));
			auto s = std::sin(node->parent()->yaw(0));
			auto const &points = area(node->parent()->trim(0), node->trim(0));
			std::vector<FLOATING_POINT_TYPE> *predicted_areas = new std::vector<FLOATING_POINT_TYPE>();
			(*predicted_areas).resize(points.size());

			for (unsigned int j = 0; j < (*predicted_areas).size(); j = j + 2) {
				(*predicted_areas)[j] = c * points[j] - s * points[j + 1] + node->parent()->x(0);
				(*predicted_areas)[j + 1] = s * points[j] + c * points[j + 1] + node->parent()->y(0);
			}
			node->shapes() = predicted_areas;
		}

		[[nodiscard]] bool check_vehicle_obstacles(Node<FLOATING_POINT_TYPE> *const node) const {
			uint16_t n_obstacles = _vehicle_obstacles[node->k()-1].size();
			for (unsigned int i = 0; i < n_obstacles; ++i) {

				if (CollisionDetection::GJK::gjk(reinterpret_cast<const CollisionDetection::vec2 *const>(node->shapes()->data()), node->shapes()->size() / 2,
				        reinterpret_cast<const CollisionDetection::vec2 *const>(_vehicle_obstacles[node->k() - 1][i].data()), _vehicle_obstacles[node->k() - 1][i].size() / 2))
					return false;
			}
			return true;
		}

		[[nodiscard]] bool check_lanelet_interaction(Node<FLOATING_POINT_TYPE> const *const node, std::vector<std::vector<CollisionDetection::vec2>> &vehicles_obstacles) const {
			if (scenario_type() == SCENARIO_TYPE::CommonRoad) {
				for (unsigned int i = 0; i < n_vehicles(); ++i) {
					double const max_x = std::max_element(vehicles_obstacles[i].begin(), vehicles_obstacles[i].end(), [](CollisionDetection::vec2 const a, CollisionDetection::vec2 const b) { return a.x < b.x; })->x;
					double const max_y = std::max_element(vehicles_obstacles[i].begin(), vehicles_obstacles[i].end(), [](CollisionDetection::vec2 const a, CollisionDetection::vec2 const b) { return a.y < b.y; })->y;
					double const min_x = std::min_element(vehicles_obstacles[i].begin(), vehicles_obstacles[i].end(), [](CollisionDetection::vec2 const a, CollisionDetection::vec2 const b) { return a.x < b.x; })->x;
					double const min_y = std::min_element(vehicles_obstacles[i].begin(), vehicles_obstacles[i].end(), [](CollisionDetection::vec2 const a, CollisionDetection::vec2 const b) { return a.y < b.y; })->y;
					for (unsigned int j = 0; j < _predicted_lanelet_boundary[i][0].size() - 2; j += 2) {
						if ((max_x < _predicted_lanelet_boundary[i][0][j] && max_x < _predicted_lanelet_boundary[i][0][j + 2]) || (min_x > _predicted_lanelet_boundary[i][0][j] && min_x > _predicted_lanelet_boundary[i][0][j + 2]) ||
						    (max_y < _predicted_lanelet_boundary[i][0][j + 1] && max_y < _predicted_lanelet_boundary[i][0][j + 3]) || (min_y > _predicted_lanelet_boundary[i][0][j + 1] && min_y > _predicted_lanelet_boundary[i][0][j + 3])) {
							continue;
						}

						if (CollisionDetection::SAT::sat(
						        (CollisionDetection::vec2 const *)vehicles_obstacles[i].data(), vehicles_obstacles[i].size() - 1, (CollisionDetection::vec2 const *)(_predicted_lanelet_boundary[i][0].data() + j), 2U))
							return false;
					}

					for (unsigned int j = 0; j < _predicted_lanelet_boundary[i][1].size() - 2; j += 2) {
						if ((max_x < _predicted_lanelet_boundary[i][1][j] && max_x < _predicted_lanelet_boundary[i][1][j + 2]) || (min_x > _predicted_lanelet_boundary[i][1][j] && min_x > _predicted_lanelet_boundary[i][1][j + 2]) ||
						    (max_y < _predicted_lanelet_boundary[i][1][j + 1] && max_y < _predicted_lanelet_boundary[i][1][j + 3]) || (min_y > _predicted_lanelet_boundary[i][1][j + 1] && min_y > _predicted_lanelet_boundary[i][1][j + 3])) {
							continue;
						}

						if (CollisionDetection::SAT::sat(
						        (CollisionDetection::vec2 const *)vehicles_obstacles[i].data(), vehicles_obstacles[i].size() - 1, (CollisionDetection::vec2 const *)(_predicted_lanelet_boundary[i][1].data() + j), 2U))
							return false;
					}
				}
			}
			return true;
		}

		[[nodiscard]] bool is_path_valid_centralized(Node<FLOATING_POINT_TYPE> const *const node) const {
			std::vector<std::vector<CollisionDetection::vec2>> vehicles_obstacles(n_vehicles());

			get_vehicle_shapes(node, vehicles_obstacles);

			// vehicle interaction for centralized search:
			for (unsigned int i = 0; i < n_vehicles(); ++i) {
				for (unsigned int j = i + 1; j < n_vehicles(); ++j) {
					if (CollisionDetection::GJK::gjk(
							(CollisionDetection::vec2 const *)vehicles_obstacles[i].data(), vehicles_obstacles[i].size() - 1, (CollisionDetection::vec2 const *)vehicles_obstacles[j].data(), vehicles_obstacles[j].size() - 1))
						return false;
				}
			}

			// TODO HDV Reachable sets against area normal offset

			// lanelet interaction:
			if(!check_lanelet_interaction(node, vehicles_obstacles)){
				return false;
			}			

			return true;
		}

		[[nodiscard]] bool eval_edge(Node<FLOATING_POINT_TYPE> *const node) const {
			std::vector<std::vector<CollisionDetection::vec2>> vehicles_obstacles(n_vehicles());

			get_vehicle_shapes(node, vehicles_obstacles);

			if (is_pb()) {
				set_predicted_areas(node);
				// other vehicle obstacles(including static obstacles) against normal offset
				if (!check_vehicle_obstacles(node)) {
					return false;
				}
			}

			// vehicle interaction for centralized search:
			/*if (!is_pb()) {
				for (unsigned int i = 0; i < n_vehicles(); ++i) {
					for (unsigned int j = i + 1; j < n_vehicles(); ++j) {
						if (CollisionDetection::GJK::gjk(
						        (CollisionDetection::vec2 const *)vehicles_obstacles[i].data(), vehicles_obstacles[i].size() - 1, (CollisionDetection::vec2 const *)vehicles_obstacles[j].data(), vehicles_obstacles[j].size() - 1))
							return false;
					}
				}
			}*/

			// TODO laneletcrossingarea against area without offset

			// TODO HDV Reachable sets against area normal offset

			// lanelet interaction:
			if (!check_lanelet_interaction(node, vehicles_obstacles)) {
				return false;
			}

			return true;
		}

		void init_new_node(Node<FLOATING_POINT_TYPE> const *const node, std::uint8_t const *const next_trims, Node<FLOATING_POINT_TYPE> *const new_node) const {
			for (unsigned int i = 0; i < n_vehicles(); ++i) {
				new_node->trim(i) = next_trims[i];

				auto c = std::cos(node->yaw(i));
				auto s = std::sin(node->yaw(i));

				auto const &dx_ = dx(node->trim(i), new_node->trim(i));
				auto const &dy_ = dy(node->trim(i), new_node->trim(i));
				auto const &dyaw_ = dyaw(node->trim(i), new_node->trim(i));

				new_node->x(i) = c * dx_ - s * dy_ + node->x(i);
				new_node->y(i) = s * dx_ + c * dy_ + node->y(i);
				new_node->yaw(i) = dyaw_ + node->yaw(i);

				// cost to come:    Distance to reference trajectory points squared to
				// conform with
				//                  J = (x-x_ref)' Q (x-x_ref)
				FLOATING_POINT_TYPE norm_x_g = new_node->x(i) - _reference_trajectory_points(i, node->k(), 0);
				FLOATING_POINT_TYPE norm_y_g = new_node->y(i) - _reference_trajectory_points(i, node->k(), 1);
				FLOATING_POINT_TYPE norm_squared_g = norm_x_g * norm_x_g + norm_y_g * norm_y_g;

				new_node->g() += norm_squared_g;

				// cost to go:      same as cost to come
				//                  subtract squared distance traveled for every
				//                  timestep and vehicle
				FLOATING_POINT_TYPE max_distance_traveled = 0.0;
				for (unsigned int j = new_node->k(); j < n_hp(); ++j) {
					max_distance_traveled += dt() * _v_ref(i, j);

					FLOATING_POINT_TYPE const norm_x_h = new_node->x(i) - _reference_trajectory_points(i, j, 0);
					FLOATING_POINT_TYPE const norm_y_h = new_node->y(i) - _reference_trajectory_points(i, j, 1);
					FLOATING_POINT_TYPE const norm_squared_h = std::pow(std::max(std::sqrt(norm_x_h * norm_x_h + norm_y_h * norm_y_h) - max_distance_traveled, 0.0), 2);

					new_node->h() += norm_squared_h;
				}
			}
		}
	};
}  // namespace GraphBasedPlanning
