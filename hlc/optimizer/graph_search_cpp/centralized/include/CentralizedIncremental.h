#pragma once

#include <IncrementalNode.h>
#include <Printer.h>
#include <Watchdog.h>
#include <vec2.h>

#include <queue>

#include "CentralizedGraphSearch.h"
#include "ConfigData.h"
#include "MPA.h"
#include "VehicleData.h"

namespace GraphBasedPlanning {
	// CentralisedIncremental recycles the already constructed tree for the next iteration. Since the cost computations are not consistent across iterations, they are determined by each node computing its own static reference points, which
	// are then used by the child nodes for the cost and heuristic computations.
	template <unsigned int n_vehicles, SCENARIO_TYPE scenario_type>
	class CentralizedIncremental : public CentralizedGraphSearch<n_vehicles, scenario_type> {
		IncrementalNode<n_vehicles> *_next_root = nullptr;
		std::priority_queue<IncrementalNode<n_vehicles> *, std::vector<IncrementalNode<n_vehicles> *>, typename IncrementalNode<n_vehicles>::priority_queue_comparison> _pq;
		unsigned int _iteration = 0;
		Watchdog &_dog;
		std::vector<int> _index_on_trajectory;
		std::uint64_t n_expanded = 0;

	   public:
		CentralizedIncremental(
		    std::shared_ptr<GraphBasedPlanning::ConfigData const> const &config, std::shared_ptr<GraphBasedPlanning::MPA const> const &mpa, std::array<VehicleData<scenario_type>, n_vehicles> const &&vehicle_data, Watchdog &dog)
		    : CentralizedGraphSearch<n_vehicles, scenario_type>(config, mpa, std::forward<decltype(vehicle_data)>(vehicle_data)), _dog(dog), _index_on_trajectory(n_vehicles, 0) {}

		Node<n_vehicles> const *search(Node<n_vehicles> const root) final {
			IncrementalNode<n_vehicles> *_root;
			auto t2 = std::chrono::high_resolution_clock::now();

			if (_iteration) {
				_root = _next_root;
				auto t0 = std::chrono::high_resolution_clock::now();

				for (unsigned int i = 0; i < _root->children_size(); ++i) {
					if (_root->child(i)) {
						_root->child(i)->g() = _root->g();
						_root->child(i)->h() = 0.0;
						init_cost(_root->child(i));
						fill_priority_queue(_root->child(i));
					}
				}

				auto t1 = std::chrono::high_resolution_clock::now();
				Printer::println("use root from before! Took ", std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count(), "ms! ");
			} else {
				_root = new IncrementalNode<n_vehicles>(root);
				Printer::print("create first root! ");

				init_incremental_trajectory_points(_root);
				expand_node(_root);
				++n_expanded;
			}

			auto t3 = std::chrono::high_resolution_clock::now();
			auto elapsed = t3 - t2;
			double seconds = std::chrono::duration<double>(elapsed).count();
#if DO_EVAL
			save(incremental_preparation, seconds);
#endif
			Printer::println((unsigned)_root->k());

			while (true) {
				IncrementalNode<n_vehicles> *const current_node = _pq.top();

				_pq.pop();

				std::array<std::vector<vec2>, n_vehicles> vehicles_obstacles;  // = this->_mpa->template calc_vehicles_obstacles_without_offset<n_vehicles>(current_node);
				if (current_node->k() < this->_config->n_hp()) {
					vehicles_obstacles = this->_mpa->template calc_vehicles_obstacles_without_offset<n_vehicles>(current_node);
				} else {
					vehicles_obstacles = this->_mpa->template calc_vehicles_obstacles_large_offset<n_vehicles>(current_node);
				}

				if (!this->is_path_valid(vehicles_obstacles)) {
					current_node->delete_from_parent();

					delete current_node;
					continue;
				}

				if (current_node->k() - _iteration >= this->_config->n_hp()) [[unlikely]] {
					init_incremental_trajectory_points(current_node);
					set_next_node(_root, current_node);

					return current_node;
				}

				init_incremental_trajectory_points(current_node);
				expand_node(current_node);
				++n_expanded;
			}
		}

		void update_vehicle_data(std::array<VehicleData<scenario_type>, n_vehicles> vehicle_data) { this->_vehicle_data = vehicle_data; }

		void clean() final {
#if DO_EVAL
			save(current_used_memory);
			save(n_expanded, n_expanded);
#endif
			n_expanded = 0;
			_pq = decltype(_pq)();  // reset queue

			_next_root->delete_from_parent();
			delete _next_root->parent();

			++_iteration;

			_dog.start();
		}

		~CentralizedIncremental() { delete _next_root; }

	   private:
		void init_incremental_trajectory_points(IncrementalNode<n_vehicles> *const node) {
			for (auto i = 0; i < n_vehicles; ++i) {
				vec2 const p = {node->x(i), node->y(i)};
				int const n_reference_trajectory_line_strips = this->_config->reference_trajectory()[i].size();

				auto [trajectory_point, min_index] = closest_point<scenario_type, 20, 20>(p, this->_config->reference_trajectory()[i], _index_on_trajectory[i]);

				if (node->k() == _iteration + this->_config->n_hp()) {
					_index_on_trajectory[i] = min_index;
				}

				unsigned int current_index = min_index + 1;
				vec2 next_trajectory_point = this->_config->reference_trajectory()[i][current_index % n_reference_trajectory_line_strips];

				// calculation of the point from which the current minimum point is speed * dt away on the trajectory
				for (unsigned int j = 0; j < this->_config->n_hp(); ++j) {
					double max_distance_still_to_be_covered = this->_config->dt() * this->_mpa->max_speed();  // * (exp(j) + 1);  // max velocity

					if (j == 0) max_distance_still_to_be_covered += this->_config->dt() * this->_mpa->max_speed();

					while (true) {
						double const distance_to_next_line_strip = distance(trajectory_point, next_trajectory_point);

						if (distance_to_next_line_strip < max_distance_still_to_be_covered) {
							max_distance_still_to_be_covered -= distance_to_next_line_strip;
							trajectory_point = next_trajectory_point;

							next_trajectory_point = this->_config->reference_trajectory()[i][++current_index % n_reference_trajectory_line_strips];

						} else {
							trajectory_point = node->trajectory_point(i, j) = get_point_along_linestrip_with_distance_from_point(trajectory_point, next_trajectory_point, max_distance_still_to_be_covered);

							break;
						}
					}
				}
			}
		}

		void init_cost(IncrementalNode<n_vehicles> *const new_node) const {
			for (unsigned int i = 0; i < n_vehicles; ++i) {
				double norm_x_g = new_node->x(i) - static_cast<IncrementalNode<n_vehicles> const *>(new_node->parent())->trajectory_point(i, 0).x;
				double norm_y_g = new_node->y(i) - static_cast<IncrementalNode<n_vehicles> const *>(new_node->parent())->trajectory_point(i, 0).y;
				double norm_squared_g = norm_x_g * norm_x_g + norm_y_g * norm_y_g;

				new_node->g() += norm_squared_g;

				double max_distance_traveled = 0.0;
				unsigned int j, k;

				for (j = new_node->k() - _iteration, k = 1; j < this->_config->n_hp(); ++j, ++k) {
					max_distance_traveled += this->_config->dt() * this->_mpa->max_speed();  // * exp(-0.0595 * j);

					double const norm_x_h = new_node->x(i) - static_cast<IncrementalNode<n_vehicles> const *>(new_node->parent())->trajectory_point(i, k).x;
					double const norm_y_h = new_node->y(i) - static_cast<IncrementalNode<n_vehicles> const *>(new_node->parent())->trajectory_point(i, k).y;
					double const norm_h = std::sqrt(norm_x_h * norm_x_h + norm_y_h * norm_y_h) - max_distance_traveled;
					double const norm_squared_h = norm_h * norm_h;

					new_node->h() += norm_squared_h;
				}
				// no recursive feasibility...
			}
		}

		void set_next_node(IncrementalNode<n_vehicles> *const root, IncrementalNode<n_vehicles> const *node) {
			for (auto i = 1; i < this->_config->n_hp(); ++i) {
				node = static_cast<IncrementalNode<n_vehicles> const *>(node->parent());
			}

			for (auto i = 0; i < root->children_size(); ++i) {
				if (root->child(i) == node) {
					_next_root = root->child(i);
				}
			}
		}

		void fill_priority_queue(IncrementalNode<n_vehicles> *node) {
			_pq.push(node);

			for (unsigned int i = 0; i < node->children_size(); ++i) {
				if (node->child(i)) {
					node->child(i)->g() = node->g();
					node->child(i)->h() = 0.0;
					init_cost(node->child(i));
					fill_priority_queue(node->child(i));
				}
			}
		}

		void expand_node(IncrementalNode<n_vehicles> *const node) {
			std::array<std::uint8_t, n_vehicles> const sizes = this->_mpa->template get_reachable_states_sizes<n_vehicles>(node->trims(), node->k() - _iteration);
			std::array<unsigned int, n_vehicles> const multipliers = this->_mpa->template get_reachable_states_multipliers<n_vehicles>(sizes);

			auto const n_children = multipliers[n_vehicles - 1] * sizes[n_vehicles - 1];

			node->create_children(n_children);

			for (auto i = 0; i < n_children; ++i) {
				std::array<std::uint8_t, n_vehicles> next_states;
				for (auto j = 0; j < n_vehicles; ++j) {
					next_states[j] = this->_mpa->reachability_list(node->trim(j))[i / multipliers[j] % sizes[j]];
				}

				auto *const new_node = new IncrementalNode<n_vehicles>(node, node->g(), node->k() + 1);
				this->_mpa->template init_node<n_vehicles>(new_node, next_states);
				init_cost(new_node);

				node->child(i) = new_node;
				_pq.push(new_node);
			}
		}
	};
}  // namespace GraphBasedPlanning