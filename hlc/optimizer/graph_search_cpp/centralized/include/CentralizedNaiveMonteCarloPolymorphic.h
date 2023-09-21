#pragma once

#include <MonteCarloNode.h>
#include <Sampler.h>

#include <boost/heap/priority_queue.hpp>
#include <memory_resource>
#include <queue>
#include <random>

#include "../../config.h"
#include "CentralizedGraphSearch.h"
#include "ConfigData.h"
#include "MPA.h"
#include "VehicleData.h"

namespace GraphBasedPlanning {
	template <unsigned int n_vehicles, SCENARIO_TYPE scenario_type, std::uint64_t n_experiments>
	class CentralizedNaiveMonteCarloPolymorphic : public CentralizedGraphSearch<n_vehicles, scenario_type>, private Sampler<> {
		std::uint64_t const size = n_experiments * sizeof(MonteCarloNode<n_vehicles>) * this->_config->n_hp() + 1024;

		boost::heap::priority_queue<MonteCarloNode<n_vehicles> const *, boost::heap::compare<typename MonteCarloNode<n_vehicles>::priority_queue_comparison>> _pq;
		std::byte *_buffer = new std::byte[size];
		std::pmr::monotonic_buffer_resource _mbr;
		std::pmr::polymorphic_allocator<> _pa{&_mbr};

	   public:
		CentralizedNaiveMonteCarloPolymorphic(
		    std::shared_ptr<GraphBasedPlanning::ConfigData const> const &config, std::shared_ptr<GraphBasedPlanning::MPA const> const &mpa, std::array<VehicleData<scenario_type>, n_vehicles> const &&vehicle_data)
		    : CentralizedGraphSearch<n_vehicles, scenario_type>(config, mpa, std::forward<decltype(vehicle_data)>(vehicle_data)), _mbr(_buffer, size) {
			_pq.reserve(n_experiments);
		}
		Node<n_vehicles> const *search(Node<n_vehicles> const root) final {
			MonteCarloNode<n_vehicles> _root = MonteCarloNode<n_vehicles>(root);
#if DO_EVAL
			auto t0 = std::chrono::high_resolution_clock::now();
#endif
			for (auto i = 0; i < n_experiments; ++i) {
				random_expand(&_root);
			}
#if DO_EVAL
			auto t1 = std::chrono::high_resolution_clock::now();
			double seconds = std::chrono::duration<double>(t1 - t0).count();
			save(expansion_time, seconds);

			std::uint64_t positives = 0;
			std::vector<std::uint64_t> where_conflict(this->_config->n_hp(), 0);
			for (auto &e : _pq) {
				if (check_path_recusive(e, where_conflict)) ++positives;
			}
			save(where_conflict, where_conflict);
			save(positives, positives);

			t0 = std::chrono::high_resolution_clock::now();
			std::uint64_t tested_for_conflict = 0;
#endif
			while (true) {
				MonteCarloNode<n_vehicles> const *const current_node = _pq.top();
#if DO_EVAL
				++tested_for_conflict;
#endif
				if (this->check_path(current_node)) {
#if DO_EVAL
					t1 = std::chrono::high_resolution_clock::now();
					seconds = std::chrono::duration<double>(t1 - t0).count();
					save(collision_detection_time, seconds);

					save(tested_for_conflict, tested_for_conflict);
#endif
					return current_node;
				}

				_pq.pop();
			}
		}
		void clean() final {
			delete this;  // commit suicide
		}

		~CentralizedNaiveMonteCarloPolymorphic() {
#if DO_EVAL
			save(current_used_memory);
#endif
			delete[] _buffer;
		}

	   private:
#if DO_EVAL
		bool check_path_recusive(MonteCarloNode<n_vehicles> const *node, std::vector<std::uint64_t> &where_conflict) const {
			if (node->k() > 1) {
				check_path_recusive(static_cast<decltype(node)>(node->parent()), where_conflict);
			}

			if (node->k() == this->_config->n_hp()) {
				if (!this->is_path_valid(this->_mpa->calc_vehicles_obstacles_large_offset(node))) {
					++where_conflict[node->k() - 1];
					return false;
				}
			} else {
				if (!this->is_path_valid(this->_mpa->calc_vehicles_obstacles_without_offset(node))) {
					++where_conflict[node->k() - 1];
					return false;
				}
			}

			return true;
		}
#endif
		void random_expand(MonteCarloNode<n_vehicles> *node) {
			std::array<std::uint8_t, n_vehicles> next_trims;

			for (auto i = 1; i <= this->_config->n_hp(); ++i) {
				for (unsigned int j = 0; j < n_vehicles; ++j) {
					std::vector<std::uint8_t> const &sampling_vector = this->_mpa->reachability_list(i - 1, node->trim(j));
					auto const index = sample(sampling_vector.size());
					next_trims[j] = sampling_vector[index];
				}

				MonteCarloNode<n_vehicles> *const new_node = _pa.new_object<MonteCarloNode<n_vehicles>>(node, node->g(), i);
				this->_mpa->template init_node<n_vehicles>(new_node, next_trims);
				this->init_cost(new_node);

				node = new_node;
			}

			_pq.push(node);
		}
	};
}  // namespace GraphBasedPlanning