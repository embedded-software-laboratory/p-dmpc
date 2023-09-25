#pragma once

#include <MonteCarloTreeSearchNodeBasic.h>
#include <Sampler.h>

#include <boost/heap/priority_queue.hpp>
#include <queue>
#include <random>

#include "CentralizedGraphSearch.h"
#include "ConfigData.h"
#include "MPA.h"
#include "VehicleData.h"

namespace GraphBasedPlanning {
	template <unsigned int n_vehicles, SCENARIO_TYPE scenario_type>
	class CentralizedMonteCarloTreeSearchBasic : public CentralizedGraphSearch<n_vehicles, scenario_type>, private Sampler<> {
		static std::uint64_t const n_shots = 1000000U;
		MonteCarloTreeSearchNodeBasic<n_vehicles> *_min;
		MonteCarloTreeSearchNodeBasic<n_vehicles> *_root = nullptr;

	   public:
		CentralizedMonteCarloTreeSearchBasic(
		    std::shared_ptr<GraphBasedPlanning::ConfigData const> const &config, std::shared_ptr<GraphBasedPlanning::MPA const> const &mpa, std::array<VehicleData<scenario_type>, n_vehicles> const &&vehicle_data)
		    : CentralizedGraphSearch<n_vehicles, scenario_type>(config, mpa, std::forward<decltype(vehicle_data)>(vehicle_data)) {
			_min = new MonteCarloTreeSearchNodeBasic<n_vehicles>(nullptr, std::numeric_limits<double>::infinity(), CentralizedGraphSearch<n_vehicles, scenario_type>::_config->n_hp(), std::numeric_limits<unsigned int>::max());
		}
		Node<n_vehicles> const *search(Node<n_vehicles> const root) final {
			_root = new MonteCarloTreeSearchNodeBasic<n_vehicles>(root);
			for (auto i = 0; i < 100000; ++i) {
				random_expand_node(_root);
			}

			return _min;
		}
		void clean() final {
			delete this;  // commit suicide
		}

		~CentralizedMonteCarloTreeSearchBasic() { delete _root; }

	   private:
		void random_expand_node(MonteCarloTreeSearchNodeBasic<n_vehicles> *const node) {
			std::array<std::uint8_t, n_vehicles> const sizes = CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->template get_reachable_states_sizes<n_vehicles>(node->trims(), node->k());
			std::array<unsigned int, n_vehicles> const multipliers = CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->template get_reachable_states_multipliers<n_vehicles>(sizes);

			if (!node->children_size()) {
				auto const n_children = multipliers[n_vehicles - 1] * sizes[n_vehicles - 1];

				node->create_children(n_children);
			}

			auto index = node->weighted_random_index();

			if (index < 0) return;

			if (node->child(index)) {
				random_expand_node(node->child(index));
			} else {
				MonteCarloTreeSearchNodeBasic<n_vehicles> *new_node = new MonteCarloTreeSearchNodeBasic<n_vehicles>(node, node->g(), node->k() + 1, index);
				auto const next_states = CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->template reachable_states<n_vehicles>(multipliers, index, node->trims(), sizes, node->k());

				CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->template init_node<n_vehicles>(new_node, next_states);
				init_cost(new_node);

				node->child(index) = new_node;

				// test:
				if (new_node->k() < CentralizedGraphSearch<n_vehicles, scenario_type>::_config->n_hp()) {
					std::array<std::vector<vec2>, n_vehicles> const vehicles_obstacles = CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->template calc_vehicles_obstacles_without_offset<n_vehicles>(new_node);
					if (!CentralizedGraphSearch<n_vehicles, scenario_type>::is_path_valid(vehicles_obstacles)) {
						node->probability(index) = 0;
					}
				} else {
					if (new_node->g() < _min->g()) {
						std::array<std::vector<vec2>, n_vehicles> const vehicles_obstacles = CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->template calc_vehicles_obstacles_without_offset<n_vehicles>(new_node);
						if (CentralizedGraphSearch<n_vehicles, scenario_type>::is_path_valid(vehicles_obstacles)) {
							_min = new_node;
						}
					}

					node->probability(index) = 0;
					return;
				}

				// Simulation: + // Backpropagation:
				if (random_simulation(new_node)) {
					for (auto i = new_node->k(); i; --i) {
						static_cast<MonteCarloTreeSearchNodeBasic<n_vehicles> *const>(new_node->parent())->probability(new_node->index()) += 1;

						new_node = static_cast<MonteCarloTreeSearchNodeBasic<n_vehicles> *const>(new_node->parent());
					}
				}
			}
		}

		bool check_path(Node<n_vehicles> const *node, unsigned int const k) const {
			if (!CentralizedGraphSearch<n_vehicles, scenario_type>::is_path_valid(CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->calc_vehicles_obstacles_large_offset(node))) {
				return false;
			}

			for (auto i = CentralizedGraphSearch<n_vehicles, scenario_type>::_config->n_hp() - 1; i > k; --i) {
				node = node->parent();

				if (!CentralizedGraphSearch<n_vehicles, scenario_type>::is_path_valid(CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->calc_vehicles_obstacles_without_offset(node))) {
					return false;
				}
			}

			return true;
		}

		void init_cost(Node<n_vehicles> *const new_node) const {
			for (auto i = 0; i < n_vehicles; ++i) {
				// cost to come:    Distance to reference trajectory points squared to
				// conform with
				//                  J = (x-x_ref)' Q (x-x_ref)
				double norm_x_g = new_node->x(i) - CentralizedGraphSearch<n_vehicles, scenario_type>::_vehicle_data[i].reference_trajectory_point(new_node->k() - 1).x;
				double norm_y_g = new_node->y(i) - CentralizedGraphSearch<n_vehicles, scenario_type>::_vehicle_data[i].reference_trajectory_point(new_node->k() - 1).y;
				double norm_squared_g = norm_x_g * norm_x_g + norm_y_g * norm_y_g;

				new_node->g() += norm_squared_g;
			}
		}

		bool random_simulation(Node<n_vehicles> *node) {
			std::array<std::uint8_t, n_vehicles> next_trims;

			auto const old_k = node->k();

			for (auto i = node->k(); i < CentralizedGraphSearch<n_vehicles, scenario_type>::_config->n_hp(); ++i) {
				for (unsigned int j = 0; j < n_vehicles; ++j) {
					std::vector<std::uint8_t> const &sampling_vector = CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->reachability_list(i, node->trim(j));
					auto const index = sample(sampling_vector.size());
					next_trims[j] = sampling_vector[index];
				}

				Node<n_vehicles> *const new_node = new Node<n_vehicles>(node, node->g(), i + 1);
				CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->template init_node<n_vehicles>(new_node, next_trims);
				init_cost(new_node);

				node = new_node;
			}

			bool ret = check_path(node, old_k);

			for (auto i = CentralizedGraphSearch<n_vehicles, scenario_type>::_config->n_hp(); i > old_k; --i) {
				auto tmp = node->parent();
				delete node;
				node = tmp;
			}

			return ret;
		}
	};
}  // namespace GraphBasedPlanning