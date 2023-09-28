#pragma once

#include <OptimalNode.h>

#include <array>
#include <queue>

#include "CentralizedGraphSearch.h"
#include "ConfigData.h"
#include "MPA.h"
#include "VehicleData.h"

namespace GraphBasedPlanning {
	// CentralizedOptimalLeafParallelization is similar to CentralizedOptimalSimple, but parallelizes each expansion process. Here, the creation of the child nodes is carried out in parallel.
	template <unsigned int n_vehicles>
	struct ThreadDataCentralizedOptimalLeafParallelization {
		std::array<std::uint8_t, n_vehicles> sizes;
		std::array<unsigned int, n_vehicles> multipliers;
		OptimalNode<n_vehicles> *node;
		unsigned int n_children;

		ThreadDataCentralizedOptimalLeafParallelization() : sizes({0}), multipliers({0}), node(nullptr), n_children(0) {}
		ThreadDataCentralizedOptimalLeafParallelization(std::array<std::uint8_t, n_vehicles> const &&sizes, std::array<unsigned int, n_vehicles> const &&multipliers, OptimalNode<n_vehicles> *const node, unsigned int const n_children)
		    : sizes(sizes), multipliers(multipliers), node(node), n_children(n_children) {}
	};

	template <unsigned int n_vehicles, SCENARIO_TYPE scenario_type, unsigned int n_threads>
	class CentralizedOptimalLeafParallelization : public CentralizedGraphSearch<n_vehicles, scenario_type> {
		OptimalNode<n_vehicles> *_root = nullptr;
		std::priority_queue<OptimalNode<n_vehicles> *, std::vector<OptimalNode<n_vehicles> *>, typename OptimalNode<n_vehicles>::priority_queue_comparison> *_pq =
		    new std::priority_queue<OptimalNode<n_vehicles> *, std::vector<OptimalNode<n_vehicles> *>, typename OptimalNode<n_vehicles>::priority_queue_comparison>;

		ThreadDataCentralizedOptimalLeafParallelization<n_vehicles> *_thread_data = new ThreadDataCentralizedOptimalLeafParallelization<n_vehicles>();
		std::vector<std::atomic<bool>> _thread_run_flag = {};
		std::vector<std::thread> _threads;
		std::atomic_flag *_data_access = new std::atomic_flag;

		void thread_function(unsigned int const id) {
			while (true) {
				while (!_thread_run_flag[id].load(std::memory_order_relaxed))  // busy wait for minimal latency
					;
				std::atomic_thread_fence(std::memory_order_acquire);

				if (!_thread_data->n_children) return;

				std::vector<OptimalNode<n_vehicles> *> queue;
				queue.reserve(_thread_data->n_children / n_threads + 1U);

				for (auto i = id; i < _thread_data->n_children; i += n_threads) {
					auto const next_states = this->_mpa->template reachable_states<n_vehicles>(_thread_data->multipliers, i, _thread_data->node->trims(), _thread_data->sizes, _thread_data->node->k());

					auto *const new_node = new OptimalNode<n_vehicles>(_thread_data->node, _thread_data->node->g(), _thread_data->node->k() + 1);
					this->_mpa->template init_node<n_vehicles>(new_node, next_states);
					this->init_cost(new_node);

					_thread_data->node->child(i) = new_node;

					if (_data_access->test_and_set()) {
						queue.push_back(new_node);
					} else {
						_pq->push(new_node);
						_data_access->clear();
					}
				}

				while (_data_access->test_and_set())
					;
				for (auto e : queue) {
					_pq->push(e);
				}
				_data_access->clear();

				std::atomic_thread_fence(std::memory_order_release);
				_thread_run_flag[id].store(false, std::memory_order_relaxed);
			}
		}

	   public:
		CentralizedOptimalLeafParallelization(
		    std::shared_ptr<GraphBasedPlanning::ConfigData const> const &config, std::shared_ptr<GraphBasedPlanning::MPA const> const &mpa, std::array<VehicleData<scenario_type>, n_vehicles> const &&vehicle_data)
		    : CentralizedGraphSearch<n_vehicles, scenario_type>(config, mpa, std::forward<decltype(vehicle_data)>(vehicle_data)), _thread_run_flag(n_threads - 1) {
			for (auto id = 0; id < n_threads - 1; ++id) {
				_thread_run_flag[id] = false;
				_threads.emplace_back(&CentralizedOptimalLeafParallelization::thread_function, this, id);
			}
			_data_access->clear();
		}
		Node<n_vehicles> const *search(Node<n_vehicles> const root) final {
			_root = new OptimalNode<n_vehicles>(root);

			expand_node(_root);

			while (true) {
				OptimalNode<n_vehicles> *current_node = _pq->top();
				_pq->pop();

				std::array<std::vector<vec2>, n_vehicles> vehicles_obstacles;
				if (current_node->k() < this->_config->n_hp()) {
					vehicles_obstacles = this->_mpa->template calc_vehicles_obstacles_without_offset<n_vehicles>(current_node);
				} else {
					vehicles_obstacles = this->_mpa->template calc_vehicles_obstacles_large_offset<n_vehicles>(current_node);
				}

				if (!this->is_path_valid(vehicles_obstacles)) {
					continue;
				}

				if (this->is_target_reached(current_node)) [[unlikely]] {
					return current_node;
				}

				expand_node(current_node);
			}
		}
		void clean() final {
#if DO_EVAL
			save(current_used_memory);
#endif
			delete this;  // commit suicide
		}

		~CentralizedOptimalLeafParallelization() {
			_thread_data->n_children = 0;

			std::atomic_thread_fence(std::memory_order_release);
			for (auto id = 0; id < n_threads - 1; ++id) {
				_thread_run_flag[id].store(true, std::memory_order_relaxed);
			}

			for (auto &e : _threads) {
				e.join();
			}
			delete _pq;
			delete _root;
			delete _thread_data;
			delete _data_access;
		}

	   private:
		void expand_node(OptimalNode<n_vehicles> *const node) {
			std::array<std::uint8_t, n_vehicles> const sizes = this->_mpa->template get_reachable_states_sizes<n_vehicles>(node->trims(), node->k());
			std::array<unsigned int, n_vehicles> const multipliers = this->_mpa->template get_reachable_states_multipliers<n_vehicles>(sizes);

			unsigned int const n_children = multipliers.back() * sizes.back();
			node->create_children(n_children);

			*_thread_data = ThreadDataCentralizedOptimalLeafParallelization<n_vehicles>(std::move(sizes), std::move(multipliers), node, n_children);

			std::atomic_thread_fence(std::memory_order_release);
			for (auto id = 0; id < n_threads - 1; ++id) {
				_thread_run_flag[id].store(true, std::memory_order_relaxed);
			}

			std::vector<OptimalNode<n_vehicles> *> queue;
			queue.reserve(_thread_data->n_children / n_threads + 1U);

			for (auto i = n_threads - 1; i < _thread_data->n_children; i += n_threads) {
				auto const next_states = this->_mpa->template reachable_states<n_vehicles>(_thread_data->multipliers, i, _thread_data->node->trims(), _thread_data->sizes, _thread_data->node->k());

				auto *const new_node = new OptimalNode<n_vehicles>(_thread_data->node, _thread_data->node->g(), _thread_data->node->k() + 1);
				this->_mpa->template init_node<n_vehicles>(new_node, next_states);
				this->init_cost(new_node);

				_thread_data->node->child(i) = new_node;

				if (_data_access->test_and_set()) {
					queue.push_back(new_node);
				} else {
					_pq->push(new_node);
					_data_access->clear();
				}
			}

			while (_data_access->test_and_set())
				;
			for (auto e : queue) {
				_pq->push(e);
			}
			_data_access->clear();

			for (auto id = 0; id < n_threads - 1; ++id) {
				while (_thread_run_flag[id].load(std::memory_order_relaxed))  // busy wait for minimal latency
					;
			}
			std::atomic_thread_fence(std::memory_order_acquire);
		}
	};
}  // namespace GraphBasedPlanning