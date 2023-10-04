#pragma once

#include <MatlabException.h>
#include <MonteCarloNode.h>
#include <Printer.h>
#include <RandomSimulationNode.h>
#include <Sampler.h>

#include <boost/heap/priority_queue.hpp>
#include <memory_resource>
#include <queue>
#include <random>
#include <vector>

#include "CentralizedGraphSearch.h"
#include "ConfigData.h"
#include "MPA.h"
#include "VehicleData.h"

namespace GraphBasedPlanning {
	// CentralizedNaiveMonteCarloLimitingSearchRange idea was to optimise the next time step or the next primitive first, so that fewer random attempts have to be made with less optimal nodes.
	template <unsigned int n_vehicles, SCENARIO_TYPE scenario_type>
	class [[deprecated("did not work too well")]] CentralizedNaiveMonteCarloLimitingSearchRange : public CentralizedGraphSearch<n_vehicles, scenario_type>, private Sampler<> {
		Node<n_vehicles> *_min;
		unsigned int shot = 0;
		unsigned int n_shots;

		std::vector<std::vector<Node<n_vehicles>>> _children;

	   public:
		CentralizedNaiveMonteCarloLimitingSearchRange(
		    std::shared_ptr<GraphBasedPlanning::ConfigData const> const &config, std::shared_ptr<GraphBasedPlanning::MPA const> const &mpa, std::array<VehicleData<scenario_type>, n_vehicles> const &&vehicle_data)
		    : CentralizedGraphSearch<n_vehicles, scenario_type>(config, mpa, std::forward<decltype(vehicle_data)>(vehicle_data)),
		      _children(CentralizedGraphSearch<n_vehicles, scenario_type>::_config->n_hp()),
		      n_shots(n_vehicles * CentralizedGraphSearch<n_vehicles, scenario_type>::_config->n_hp()) /* dynamically increase max shots */ {
			_min = new MonteCarloNode<n_vehicles>(nullptr, std::numeric_limits<double>::infinity(), CentralizedGraphSearch<n_vehicles, scenario_type>::_config->n_hp(), std::numeric_limits<unsigned int>::max());

			// omp_set_num_threads(4);
		}

		std::integral auto limit_search_range(Node<n_vehicles> *const node) {
			std::unsigned_integral auto const k = node->k();

			std::array<std::uint8_t, n_vehicles> const sizes = CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->template get_reachable_states_sizes<n_vehicles>(node->trims(), k);
			std::array<unsigned int, n_vehicles> const multipliers = CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->template get_reachable_states_multipliers<n_vehicles>(sizes);
			auto n_children = multipliers[n_vehicles - 1] * sizes[n_vehicles - 1];

			_children[k] = std::vector<Node<n_vehicles>>(n_children, Node<n_vehicles>(node, node->g(), k + 1));

			// construct children of current focused node:
			std::signed_integral auto child = 0;
			for (std::signed_integral auto i = 0; i < n_children; ++i, ++child) {
				auto const next_states = CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->template reachable_states<n_vehicles>(multipliers, i, node->trims(), sizes, k);
				CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->template init_node<n_vehicles>(&_children[k][child], next_states);

				if (CentralizedGraphSearch<n_vehicles, scenario_type>::is_path_valid(CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->calc_vehicles_obstacles_without_offset(&_children[k][child]))) [[likely]] {
					init_cost(&_children[k][child]);
				} else [[unlikely]] {
					// don't need a delete because init_node replace specific values of former node
					--child;
					// _children[node->k()].pop_back();
				}
			}
			n_children = child;

			unsigned int index;
			std::vector<double> cumulative_cost_to_come(n_children, 0.0);
			std::vector<double> n_attempts(n_children, 0.0);
			std::vector<double> average_cost_to_come(n_children, 0.0);
			do {
				// #pragma omp parallel for default(none) shared(_children, cumulative_cost_to_come, n_attempts) firstprivate(n_children, k)
				for (auto i = 0; i < 25; ++i) {
					for (auto j = 0; j < n_children; ++j) {
						random_expand(&_children[k][j], cumulative_cost_to_come[j], n_attempts[j]);
					}
				}

				for (auto i = 0; i < cumulative_cost_to_come.size(); ++i) {
					average_cost_to_come[i] = cumulative_cost_to_come[i] / n_attempts[i];  // 0.0 / 0.0 would result in -nan -> cant be min
				}

				index = std::distance(average_cost_to_come.begin(), std::min_element(average_cost_to_come.begin(), average_cost_to_come.end()));
				if (std::none_of(_children[k][index].trims().begin(), _children[k][index].trims().end(), [](std::uint8_t i) { return i == 0; })) break;
			} while (++shot < n_shots);

			for (auto e : _children[k][index].trims()) Printer::print((unsigned int)e, ", ");
			Printer::println();

			return index;
		}

		Node<n_vehicles> const *search(Node<n_vehicles> const root) final {
			MonteCarloNode<n_vehicles> _root = MonteCarloNode<n_vehicles>(root);

			auto index = limit_search_range(&_root);
			for (auto i = 1; i < CentralizedGraphSearch<n_vehicles, scenario_type>::_config->n_hp() - 1; ++i) {
				if (shot < n_shots) index = limit_search_range(&_children[i - 1][index]);
			}

			if (_min->g() == std::numeric_limits<double>::infinity()) {
				throw MatlabException("NO Solution found!");
			}

			return _min;
		}
		void clean() final {
			delete this;  // commit suicide
		}

		~CentralizedNaiveMonteCarloLimitingSearchRange() { delete _min; }

	   private:
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

		void random_expand(Node<n_vehicles> *node, double &cost, double &attempts) {
			auto const old_k = node->k();

			for (auto i = old_k; i < CentralizedGraphSearch<n_vehicles, scenario_type>::_config->n_hp(); ++i) {
				std::array<std::uint8_t, n_vehicles> next_trims;

				for (unsigned int j = 0; j < n_vehicles; ++j) {
					std::vector<std::uint8_t> const &sampling_vector = CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->reachability_list(i, node->trim(j));
					auto const index = sample(sampling_vector.size());
					next_trims[j] = sampling_vector[index];
				}

				RandomSimulationNode<n_vehicles> *const new_node = new RandomSimulationNode<n_vehicles>(node, node->g(), i + 1, i == old_k);
				CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->template init_node<n_vehicles>(new_node, next_trims);
				init_cost(new_node);

				node = new_node;
			}

			bool valid = check_path(node, old_k);

			if (!valid) {
				delete node;
				return;
			}
			// #pragma omp critical
			//			{
			cost += node->g();
			attempts += 1.0;

			if (node->g() < _min->g()) {
				delete _min;
				_min = node;
			} else {
				delete node;
			}
		}
		//		}
	};
}  // namespace GraphBasedPlanning