#pragma once

#include <OptimalNode.h>

#include <atomic>
#include <barrier>
#include <memory_resource>
#include <queue>

#include "CentralizedGraphSearch.h"
#include "ConfigData.h"
#include "MPA.h"
#include "VehicleData.h"

namespace GraphBasedPlanning {
	// CentralizedOptimalRootParallelization is similar to CentralizedOptimalSimple, but parallelizes the A* execution. Here, several node expansions can be carried out concurrently.
	template <unsigned int n_vehicles, SCENARIO_TYPE scenario_type, unsigned int n_threads>
	class CentralizedOptimalRootParallelization : public CentralizedGraphSearch<n_vehicles, scenario_type> {
		OptimalNode<n_vehicles> *_root = nullptr;
		std::priority_queue<OptimalNode<n_vehicles> *, std::vector<OptimalNode<n_vehicles> *>, typename OptimalNode<n_vehicles>::priority_queue_comparison> *_pq =
		    new std::priority_queue<OptimalNode<n_vehicles> *, std::vector<OptimalNode<n_vehicles> *>, typename OptimalNode<n_vehicles>::priority_queue_comparison>;
		std::atomic_flag *_data_access = new std::atomic_flag;

		std::vector<std::atomic_flag> _thread_stop_flag;
		bool _found = false;
		OptimalNode<n_vehicles> const *_solution = nullptr;
		int _attempts = 0;

		std::array<std::atomic<OptimalNode<n_vehicles> *>, n_threads> _current_nodes = {nullptr};
		// std::vector<OptimalNode<n_vehicles> *> _current_nodes;
#if DO_EVAL
		std::atomic<std::uint64_t> n_expanded = 0;
#endif

		template <typename CompletionFunction>
		void thread_function(OptimalNode<n_vehicles> const **const result, unsigned int const id, std::barrier<CompletionFunction> &sync_point) {
			do {
				while (_thread_stop_flag[id].test_and_set()) {
					OptimalNode<n_vehicles> *current_node;

					if (!_current_nodes[id].load()) {
						while (_data_access->test_and_set())
							;

						auto value = _pq->top();
						_pq->pop();

						for (auto i = 0; i < id; ++i) {
							OptimalNode<n_vehicles> *tmp = nullptr;
							if (_current_nodes[i].compare_exchange_strong(tmp, value)) {
								value = _pq->top();
								_pq->pop();
							}
						}

						for (auto i = id + 1; i < _current_nodes.size(); ++i) {
							OptimalNode<n_vehicles> *tmp = nullptr;
							if (_current_nodes[i].compare_exchange_strong(tmp, value)) {
								value = _pq->top();
								_pq->pop();
							}
						}

						// for (auto &e : _current_nodes) {
						//	if (!e) {
						//		e = _pq->top();
						//		_pq->pop();
						//	}
						// }

						_data_access->clear();

						current_node = value;
					} else {
						current_node = _current_nodes[id].exchange(nullptr);
					}

					// OptimalNode<n_vehicles> *current_node = _current_nodes[id];
					//_current_nodes[id] = nullptr;

					std::array<std::vector<vec2>, n_vehicles> vehicles_obstacles;
					if (current_node->k() < this->_config->n_hp()) {
						vehicles_obstacles = this->_mpa->template calc_vehicles_obstacles_without_offset<n_vehicles>(current_node);
					} else {
						vehicles_obstacles = this->_mpa->template calc_vehicles_obstacles_large_offset<n_vehicles>(current_node);
					}

					if (!this->is_path_valid(vehicles_obstacles)) continue;

					if (this->is_target_reached(current_node)) [[unlikely]] {
						*result = current_node;

						for (auto &e : _thread_stop_flag) {  // make the other threads _thread_stop_flag
							e.clear();
						}

						break;
					}

					expand_node(current_node);
#if DO_EVAL
					++n_expanded;
#endif
				}
				sync_point.arrive_and_wait();
				_thread_stop_flag[id].test_and_set();
			} while (!_found);
		}

	   public:
		CentralizedOptimalRootParallelization(
		    std::shared_ptr<GraphBasedPlanning::ConfigData const> const &config, std::shared_ptr<GraphBasedPlanning::MPA const> const &mpa, std::array<VehicleData<scenario_type>, n_vehicles> const &&vehicle_data)
		    : CentralizedGraphSearch<n_vehicles, scenario_type>(config, mpa, std::forward<decltype(vehicle_data)>(vehicle_data)), _thread_stop_flag(n_threads) {
			for (auto &e : _thread_stop_flag) {
				e.test_and_set();
			}
			_data_access->clear();
		}
		Node<n_vehicles> const *search(Node<n_vehicles> const root) final {
			_root = new OptimalNode<n_vehicles>(root);
			expand_node(_root);
#if DO_EVAL
			++n_expanded;
#endif

			std::array<OptimalNode<n_vehicles> const *, n_threads> results;
			OptimalNode<n_vehicles> dummy_worst_node(nullptr, std::numeric_limits<double>::infinity(), this->_config->n_hp());
			std::fill(results.begin(), results.end(), &dummy_worst_node);

			auto on_completion = [this, &results]() noexcept {
				_solution = *std::min_element(results.begin(), results.end(), [](OptimalNode<n_vehicles> const *const lhs, OptimalNode<n_vehicles> const *const rhs) { return lhs->g() + lhs->h() < rhs->g() + rhs->h(); });

				for (auto &e : _current_nodes) {
					auto el = e.load();
					if (el) {
						_pq->push(el);
						e.store(nullptr);
					}
				}

				// Unfortunately, this will return wrong result for some instances -> because heuristic is not underestimating!!! change get_predicted_lanelets v_ref or sample_reference_trajectory computation
				OptimalNode<n_vehicles> *current_node = _pq->top();
				_pq->pop();

				if (_solution->g() + _solution->h() < current_node->g() + current_node->h()) {
					_found = true;
				} else {
					_pq->push(current_node);
					++_attempts;
				}
			};

			std::barrier<decltype(on_completion)> sync_point(n_threads, on_completion);
			std::vector<std::thread> tasks;

			unsigned int id;
			for (id = 0; id < n_threads - 1; ++id) {
				tasks.emplace_back(&CentralizedOptimalRootParallelization::thread_function<decltype(on_completion)>, this, &results[id], id, std::ref(sync_point));
			}

			thread_function<decltype(on_completion)>(&results[id], id, std::ref(sync_point));

			for (auto &e : tasks) {
				e.join();
			}

			Printer::println("attempts: ", _attempts);
			return *std::min_element(results.begin(), results.end(), [](OptimalNode<n_vehicles> const *const lhs, OptimalNode<n_vehicles> const *const rhs) { return lhs->g() + lhs->h() < rhs->g() + rhs->h(); });
		}
		void clean() final {
#if DO_EVAL
			save(current_used_memory);
			save(n_expanded, n_expanded);
#endif
			delete this;  // commit suicide
		}

		~CentralizedOptimalRootParallelization() {
			delete _root;
			delete _pq;
			delete _data_access;
		}

	   private:
		void expand_node(OptimalNode<n_vehicles> *const node) {
			std::array<std::uint8_t, n_vehicles> const sizes = this->_mpa->template get_reachable_states_sizes<n_vehicles>(node->trims(), node->k());
			std::array<unsigned int, n_vehicles> const multipliers = this->_mpa->template get_reachable_states_multipliers<n_vehicles>(sizes);

			auto const n_children = multipliers.back() * sizes.back();
			node->create_children(n_children);
			std::vector<OptimalNode<n_vehicles> *> children(n_children);

			for (auto i = 0; i < n_children; ++i) {
				auto const next_states = this->_mpa->template reachable_states<n_vehicles>(multipliers, i, node->trims(), sizes, node->k());

				auto *const new_node = new OptimalNode<n_vehicles>(node, node->g(), node->k() + 1);
				this->_mpa->template init_node<n_vehicles>(new_node, next_states);
				this->init_cost(new_node);

				node->child(i) = new_node;
				children[i] = new_node;
			}

			while (_data_access->test_and_set())
				;

			for (auto e : children) {
				_pq->push(e);
			}
			_data_access->clear();
		}
	};
}  // namespace GraphBasedPlanning