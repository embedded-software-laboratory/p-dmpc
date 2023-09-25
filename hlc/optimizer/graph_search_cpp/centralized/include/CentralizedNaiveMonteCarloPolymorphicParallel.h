#pragma once

#include <MonteCarloNode.h>
#include <Sampler.h>

#include <boost/heap/priority_queue.hpp>
#include <cstdint>
#include <memory_resource>
#include <queue>
#include <random>
#include <thread>
#include <utility>

#include "CentralizedGraphSearch.h"
#include "ConfigData.h"
#include "MPA.h"
#include "VehicleData.h"

namespace GraphBasedPlanning {
	template <unsigned int n_vehicles, unsigned int n_threads, std::uint64_t n_experiments>
	struct ThreadDataCentralizedNaiveMonteCarloPolymorphicParallel {
		boost::heap::priority_queue<MonteCarloNode<n_vehicles> const *, boost::heap::compare<typename MonteCarloNode<n_vehicles>::priority_queue_comparison>> pq;
		std::byte *buffer;
		std::pmr::monotonic_buffer_resource mbr;
		std::pmr::polymorphic_allocator<> pa{&mbr};

		explicit ThreadDataCentralizedNaiveMonteCarloPolymorphicParallel(std::uint64_t const size) : buffer(new std::byte[size]), mbr(buffer, size) { pq.reserve(n_experiments / n_threads); }

		~ThreadDataCentralizedNaiveMonteCarloPolymorphicParallel() { delete[] buffer; }
	};

	template <unsigned int n_vehicles, SCENARIO_TYPE scenario_type, std::uint64_t n_experiments, unsigned int n_threads>
	class CentralizedNaiveMonteCarloPolymorphicParallel : public CentralizedGraphSearch<n_vehicles, scenario_type>, private Sampler<> {
		std::array<ThreadDataCentralizedNaiveMonteCarloPolymorphicParallel<n_vehicles, n_threads, n_experiments>, n_threads> _thread_data;

		std::array<ThreadDataCentralizedNaiveMonteCarloPolymorphicParallel<n_vehicles, n_threads, n_experiments>, n_threads> make_thread_data() {
			std::uint64_t const size = (n_experiments / n_threads) * sizeof(MonteCarloNode<n_vehicles>) * this->_config->n_hp() + 1024;
			auto produce_thread_data = [size]<std::uint64_t... I>(std::integer_sequence<std::uint64_t, I...>) -> std::array<ThreadDataCentralizedNaiveMonteCarloPolymorphicParallel<n_vehicles, n_threads, n_experiments>, n_threads> {
				return {ThreadDataCentralizedNaiveMonteCarloPolymorphicParallel<n_vehicles, n_threads, n_experiments>(size + I * 0)...};
			};

			return produce_thread_data(std::make_integer_sequence<std::uint64_t, n_threads>{});
		}

		void thread_function(MonteCarloNode<n_vehicles> const **const result, MonteCarloNode<n_vehicles> root, unsigned int const id) {
			for (auto i = 0; i < (n_experiments / n_threads); ++i) {
				random_expand(&root, id);
			}

			while (!_thread_data[id].pq.empty()) {
				MonteCarloNode<n_vehicles> const *const current_node = _thread_data[id].pq.top();

				if (this->check_path(current_node)) {
					*result = current_node;
					return;
				}

				_thread_data[id].pq.pop();
			}
		}

	   public:
		CentralizedNaiveMonteCarloPolymorphicParallel(
		    std::shared_ptr<GraphBasedPlanning::ConfigData const> const &config, std::shared_ptr<GraphBasedPlanning::MPA const> const &mpa, std::array<VehicleData<scenario_type>, n_vehicles> const &&vehicle_data)
		    : CentralizedGraphSearch<n_vehicles, scenario_type>(config, mpa, std::forward<decltype(vehicle_data)>(vehicle_data)), _thread_data(make_thread_data()) {}
		Node<n_vehicles> const *search(Node<n_vehicles> const root) final {
			std::array<MonteCarloNode<n_vehicles> const *, n_threads> results;
			std::fill(results.begin(), results.end(), nullptr);
			std::vector<std::thread> tasks;

			unsigned int id;
			for (id = 0; id < n_threads - 1; ++id) {
				tasks.emplace_back(&CentralizedNaiveMonteCarloPolymorphicParallel::thread_function, this, &results[id], MonteCarloNode<n_vehicles>(root), id);
			}

			thread_function(&results[id], MonteCarloNode<n_vehicles>(root), id);

			for (auto &e : tasks) {
				e.join();
			}

			MonteCarloNode<n_vehicles> const *res = nullptr;
			for (bool one_found = false; auto e : results) {
				if (e == nullptr) continue;
				if (!one_found) {
					one_found = true;
					res = e;
				}
				if (e->g() < res->g()) {
					res = e;
				}
			}
			if (!res) throw MatlabException("no solution found!");

			return res;
		}
		void clean() final {
#if DO_EVAL
			save(current_used_memory);
#endif
			delete this;  // commit suicide
		}

	   private:
		void random_expand(MonteCarloNode<n_vehicles> *node, unsigned int const id) {
			std::array<std::uint8_t, n_vehicles> next_trims;

			for (auto i = 1; i <= this->_config->n_hp(); ++i) {
				for (unsigned int j = 0; j < n_vehicles; ++j) {
					std::vector<std::uint8_t> const &sampling_vector = this->_mpa->reachability_list(i - 1, node->trim(j));
					auto const index = sample(sampling_vector.size());
					next_trims[j] = sampling_vector[index];
				}

				MonteCarloNode<n_vehicles> *const new_node = _thread_data[id].pa.template new_object<MonteCarloNode<n_vehicles>>(node, node->g(), i);
				this->_mpa->template init_node<n_vehicles>(new_node, next_trims);
				this->init_cost(new_node);

				node = new_node;
			}

			_thread_data[id].pq.push(node);
		}
	};
}  // namespace GraphBasedPlanning