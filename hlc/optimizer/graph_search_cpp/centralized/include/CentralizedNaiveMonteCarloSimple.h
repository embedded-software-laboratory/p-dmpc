#pragma once

#include <MonteCarloNode.h>
#include <Sampler.h>

#include <boost/heap/priority_queue.hpp>
#include <queue>

#include "../../config.h"
#include "CentralizedGraphSearch.h"
#include "ConfigData.h"
#include "MPA.h"
#include "VehicleData.h"

namespace GraphBasedPlanning {
	// CentralizedNaiveMonteCarloSimple is similar to CentralisedNaiveMonteCarloPolymorphic, but don't use polymorphic memory resource.
	template <unsigned int n_vehicles, SCENARIO_TYPE scenario_type, std::uint64_t n_experiments>
	class CentralizedNaiveMonteCarloSimple : public CentralizedGraphSearch<n_vehicles, scenario_type>, private Sampler<> {
		boost::heap::priority_queue<MonteCarloNode<n_vehicles> const *, boost::heap::compare<typename MonteCarloNode<n_vehicles>::priority_queue_comparison>> _pq;

	   public:
		CentralizedNaiveMonteCarloSimple(
		    std::shared_ptr<GraphBasedPlanning::ConfigData const> const &config, std::shared_ptr<GraphBasedPlanning::MPA const> const &mpa, std::array<VehicleData<scenario_type>, n_vehicles> const &&vehicle_data)
		    : CentralizedGraphSearch<n_vehicles, scenario_type>(config, mpa, std::forward<decltype(vehicle_data)>(vehicle_data)) {
			_pq.reserve(n_experiments);
		}
		Node<n_vehicles> const *search(Node<n_vehicles> const root) final {
			MonteCarloNode<n_vehicles> _root = MonteCarloNode<n_vehicles>(root);

			for (auto i = 0; i < n_experiments; ++i) {
				random_expand(&_root);
			}

			while (true) {
				MonteCarloNode<n_vehicles> const *const current_node = _pq.top();

				if (this->check_path(current_node)) return current_node;

				_pq.pop();
				delete_node_and_parents(current_node);
			}
		}
		void clean() final {
#if DO_EVAL
			save(current_used_memory);
#endif
			delete this;  // commit suicide
		}

		~CentralizedNaiveMonteCarloSimple() {
			for (auto e : _pq) {
				delete_node_and_parents(e);
			}
		}

	   private:
		void delete_node_and_parents(MonteCarloNode<n_vehicles> const *node) const {  // without root
			for (auto i = 0; i < this->_config->n_hp(); ++i) {
				auto tmp = static_cast<decltype(node)>(node->parent());
				delete node;
				node = tmp;
			}
		}

		void random_expand(MonteCarloNode<n_vehicles> *node) {
			std::array<std::uint8_t, n_vehicles> next_trims;

			for (auto i = 1; i <= this->_config->n_hp(); ++i) {
				for (unsigned int j = 0; j < n_vehicles; ++j) {
					std::vector<std::uint8_t> const &sampling_vector = this->_mpa->reachability_list(i - 1, node->trim(j));
					auto const index = sample(sampling_vector.size());
					next_trims[j] = sampling_vector[index];
				}

				auto *const new_node = new MonteCarloNode<n_vehicles>(node, node->g(), i);
				this->_mpa->template init_node<n_vehicles>(new_node, next_trims);
				this->init_cost(new_node);

				node = new_node;
			}

			_pq.push(node);
		}
	};
}  // namespace GraphBasedPlanning