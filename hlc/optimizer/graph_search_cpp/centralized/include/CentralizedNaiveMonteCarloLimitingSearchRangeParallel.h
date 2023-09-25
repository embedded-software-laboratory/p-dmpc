#pragma once

#include <MatlabException.h>
#include <MonteCarloNode.h>
#include <Printer.h>
#include <RandomSimulationNode.h>
#include <Sampler.h>

#include <array>
#include <barrier>
#include <boost/heap/priority_queue.hpp>
#include <memory_resource>
#include <queue>
#include <semaphore>
#include <vector>

#include "CentralizedGraphSearch.h"
#include "ConfigData.h"
#include "MPA.h"
#include "VehicleData.h"

namespace GraphBasedPlanning {

	template <unsigned int n_vehicles>
	struct ThreadDataCentralizedNaiveMonteCarloLimitingSearchRangeParallel {
		std::vector<std::vector<Node<n_vehicles>>> _children;

		ThreadDataCentralizedNaiveMonteCarloLimitingSearchRangeParallel(auto const n_level) : _children(n_level) {}
	};

	template <unsigned int n_vehicles, SCENARIO_TYPE scenario_type, unsigned int n_threads>
	class CentralizedNaiveMonteCarloLimitingSearchRangeParallel : public CentralizedGraphSearch<n_vehicles, scenario_type>, private Sampler<> {
		Node<n_vehicles> *_min;
		unsigned int shot = 0;
		unsigned int n_shots;

		std::array<ThreadDataCentralizedNaiveMonteCarloLimitingSearchRangeParallel<n_vehicles>, n_threads> _thread_data;
		std::array<Node<n_vehicles> *, n_threads> nodes_with_best_average_cost;
		Node<n_vehicles> *node_with_best_average_cost;
		bool redo = false;
		std::binary_semaphore check_node{1};

		template <typename CompletionFunction>
		void thread_function(unsigned int const id, Node<n_vehicles> *const node, Node<n_vehicles> **const node_with_best_average_cost_thread, std::barrier<CompletionFunction> &sync_point) {
			std::unsigned_integral auto const k = node->k();

			std::array<std::uint8_t, n_vehicles> const sizes = CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->template get_reachable_states_sizes<n_vehicles>(node->trims(), k);
			std::array<unsigned int, n_vehicles> const multipliers = CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->template get_reachable_states_multipliers<n_vehicles>(sizes);
			auto const n_children = multipliers[n_vehicles - 1] * sizes[n_vehicles - 1];

			auto const n_children_thread = n_children % n_threads ? n_children / n_threads + 1 : n_children / n_threads;

			_thread_data[id]._children[k] = std::vector<Node<n_vehicles>>(n_children_thread, Node<n_vehicles>(node, node->g(), k + 1));

			// construct children of current focused node:
			std::signed_integral auto n_children_thread_real = 0;
			for (auto i = id * n_children_thread; i < std::min((id + 1) * n_children_thread, n_children); ++i, ++n_children_thread_real) {
				auto const next_states = CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->template reachable_states<n_vehicles>(multipliers, i, node->trims(), sizes, k);
				CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->template init_node<n_vehicles>(&_thread_data[id]._children[k][n_children_thread_real], next_states);

				if (CentralizedGraphSearch<n_vehicles, scenario_type>::is_path_valid(CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->calc_vehicles_obstacles_without_offset(&_thread_data[id]._children[k][n_children_thread_real])))
				    [[likely]] {
					init_cost(&_thread_data[id]._children[k][n_children_thread_real]);
				} else [[unlikely]] {
					// don't need a delete because init_node replace specific values of former node
					--n_children_thread_real;
					// _children[k].pop_back();
				}
			}

			std::vector<double> cumulative_cost_to_come(n_children_thread_real, 0.0);
			std::vector<double> n_attempts(n_children_thread_real, 0.0);
			std::vector<double> average_cost_to_come(n_children_thread_real, 0.0);
			do {
				for (auto i = 0; i < 100; ++i) {
					for (auto j = 0; j < n_children_thread_real; ++j) {
						random_expand(&_thread_data[id]._children[k][j], cumulative_cost_to_come[j], n_attempts[j]);
					}
				}

				for (auto i = 0; i < n_children_thread_real; ++i) {
					average_cost_to_come[i] = cumulative_cost_to_come[i] / n_attempts[i];  // 0.0 / 0.0 would result in -nan -> cant be min
				}

				auto index = std::distance(average_cost_to_come.begin(), std::min_element(average_cost_to_come.begin(), average_cost_to_come.end()));
				*node_with_best_average_cost_thread = &_thread_data[id]._children[k][index];

				sync_point.arrive_and_wait();
			} while (redo);
		}

		std::array<ThreadDataCentralizedNaiveMonteCarloLimitingSearchRangeParallel<n_vehicles>, n_threads> make_thread_data() {
			unsigned int const n_level = CentralizedGraphSearch<n_vehicles, scenario_type>::_config->n_hp() - 2;
			auto produce_thread_data_array = [n_level]<unsigned int... I>(std::integer_sequence<unsigned int, I...>) -> std::array<ThreadDataCentralizedNaiveMonteCarloLimitingSearchRangeParallel<n_vehicles>, n_threads> {
				return {ThreadDataCentralizedNaiveMonteCarloLimitingSearchRangeParallel<n_vehicles>(n_level + I * 0)...};
			};

			return produce_thread_data_array(std::make_integer_sequence<unsigned int, n_threads>{});
		}

	   public:
		CentralizedNaiveMonteCarloLimitingSearchRangeParallel(
		    std::shared_ptr<GraphBasedPlanning::ConfigData const> const &config, std::shared_ptr<GraphBasedPlanning::MPA const> const &mpa, std::array<VehicleData<scenario_type>, n_vehicles> const &&vehicle_data)
		    : CentralizedGraphSearch<n_vehicles, scenario_type>(config, mpa, std::forward<decltype(vehicle_data)>(vehicle_data)),
		      _thread_data(make_thread_data()),
		      n_shots(n_vehicles * CentralizedGraphSearch<n_vehicles, scenario_type>::_config->n_hp()) /* dynamically increase max shots */ {
			_min = new MonteCarloNode<n_vehicles>(nullptr, std::numeric_limits<double>::infinity(), CentralizedGraphSearch<n_vehicles, scenario_type>::_config->n_hp(), std::numeric_limits<unsigned int>::max());
		}

		Node<n_vehicles> const *search(Node<n_vehicles> const root) final {
			Node<n_vehicles> *node = new MonteCarloNode<n_vehicles>(root);

			auto on_completion = [this]() noexcept {
				node_with_best_average_cost =
				    *std::min_element(nodes_with_best_average_cost.begin(), nodes_with_best_average_cost.end(), [](Node<n_vehicles> const *const lhs, Node<n_vehicles> const *const rhs) { return lhs->g() < rhs->g(); });

				redo = ++shot < n_shots && std::any_of(node_with_best_average_cost->trims().begin(), node_with_best_average_cost->trims().end(), [](std::uint8_t const i) { return i == 0; });
			};
			std::barrier<decltype(on_completion)> sync_point(n_threads, on_completion);

			for (auto i = 1; i < CentralizedGraphSearch<n_vehicles, scenario_type>::_config->n_hp() - 1; ++i) {
				if (shot < n_shots) {
					std::vector<std::thread> threads;
					threads.reserve(n_threads - 1);
					auto id = 0;
					for (; id < n_threads - 1; ++id) {
						threads.emplace_back(&CentralizedNaiveMonteCarloLimitingSearchRangeParallel::thread_function<decltype(on_completion)>, this, id, node, &nodes_with_best_average_cost[id], std::ref(sync_point));
					}
					thread_function<decltype(on_completion)>(id, node, &nodes_with_best_average_cost[id], std::ref(sync_point));
					for (auto &thread : threads) thread.join();

					for (auto e : node_with_best_average_cost->trims()) Printer::print((unsigned int)e, ", ");
					Printer::println();

					if (i == 1) delete node;
					node = node_with_best_average_cost;
				}
			}

			if (_min->g() == std::numeric_limits<double>::infinity()) {
				throw MatlabException("NO Solution found!");
			}

			return _min;
		}
		void clean() final {
			delete this;  // commit suicide
		}

		~CentralizedNaiveMonteCarloLimitingSearchRangeParallel() { delete _min; }

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

			cost += node->g();
			attempts += 1.0;

			check_node.acquire();
			if (node->g() < _min->g()) {
				auto tmp = _min;
				_min = node;
				check_node.release();
				delete tmp;
			} else {
				check_node.release();
				delete node;
			}
		}
	};
}  // namespace GraphBasedPlanning