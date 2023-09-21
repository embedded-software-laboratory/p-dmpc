#pragma once

#include <OptimalNode.h>

#include <array>
#include <queue>
#include <set>
#include <tuple>

#include "CentralizedGraphSearch.h"
#include "CentralizedGroupingConflictBased.h"
#include "CentralizedNaiveMonteCarloPolymorphic.h"
#include "CentralizedOptimalPolymorphic.h"
#include "ConfigData.h"
#include "CouplingData.h"
#include "MPA.h"
#include "VehicleData.h"

namespace GraphBasedPlanning {
	template <unsigned int n_vehicles, SCENARIO_TYPE scenario_type>
	class CentralizedGrouping : public CentralizedGraphSearch<n_vehicles, scenario_type> {
		std::vector<std::set<unsigned int>> _coupling;

		Node<n_vehicles> *solution = nullptr;

	   public:
		CentralizedGrouping(std::shared_ptr<GraphBasedPlanning::ConfigData const> const &config, std::shared_ptr<GraphBasedPlanning::MPA const> const &mpa, std::array<VehicleData<scenario_type>, n_vehicles> const &&vehicle_data,
		    CouplingData const &coupling_data)
		    : CentralizedGraphSearch<n_vehicles, scenario_type>(config, mpa, std::forward<decltype(vehicle_data)>(vehicle_data)), _coupling(calculate_together(coupling_data)) {}

		Node<n_vehicles> const *search(Node<n_vehicles> const root) final {
			solution = new Node<n_vehicles>(nullptr, 0.0, 1);
			for (auto k = 2; k <= CentralizedGraphSearch<n_vehicles, scenario_type>::_config->n_hp(); ++k) {
				solution = new Node<n_vehicles>(solution, 0.0, k);
			}

			for (auto &e : _coupling) {
				switch (e.size()) {
					case 1: search<1>(root, e); break;
					case 2: search<2>(root, e); break;
					case 3: search<3>(root, e); break;
					case 4: search<4>(root, e); break;
					case 5: search<5>(root, e); break;
					case 6: search<6>(root, e); break;
					default: throw MatlabException("Graph Search with '", e.size(), "' vehicles not implemented/available!"); break;
				}
			}

			return solution;
		}
		void clean() final {
			delete this;  // commit suicide
		}

		~CentralizedGrouping() { delete solution; }

	   private:
		template <unsigned int partial_n_vehicles>
		void search(Node<n_vehicles> const root, std::set<unsigned int> &indices) {
			auto partial_root = produce_root<partial_n_vehicles>(root, indices);

			try {
				auto partial_vehicle_data = produce_new_vehicle_data<partial_n_vehicles>(this->_vehicle_data, indices);
				auto graph_search = CentralizedGroupingConflictBased<partial_n_vehicles, scenario_type>(this->_config, this->_mpa, std::move(partial_vehicle_data));
				convert_nodes_to_single(graph_search.search(partial_root), indices);
			} catch (NoSolutionException &e) {
				if constexpr (partial_n_vehicles == 1) throw MatlabException("partial_n_vehicles == 1");
				auto partial_vehicle_data = produce_new_vehicle_data<partial_n_vehicles>(this->_vehicle_data, indices);
				if constexpr (partial_n_vehicles <= 3) {
					auto graph_search = CentralizedOptimalPolymorphic<partial_n_vehicles, scenario_type>(this->_config, this->_mpa, std::move(partial_vehicle_data));
					convert_nodes_to_single(graph_search.search(partial_root), indices);
				} else {
					auto graph_search = CentralizedNaiveMonteCarloPolymorphic<partial_n_vehicles, scenario_type, Experiments>(this->_config, this->_mpa, std::move(partial_vehicle_data));
					convert_nodes_to_single(graph_search.search(partial_root), indices);
				}
			}
		}

		template <std::size_t N>
		std::array<VehicleData<scenario_type>, N> produce_new_vehicle_data(std::array<VehicleData<scenario_type>, n_vehicles> const &vehicle_data, std::set<unsigned int> const &indices) {
			auto produce_new_vehicle_data_impl = [vehicle_data, indices]<std::size_t... I>(std::index_sequence<I...>) -> std::array<VehicleData<scenario_type>, N> { return {vehicle_data[*std::next(indices.begin(), I)]...}; };
			return produce_new_vehicle_data_impl(std::make_index_sequence<N>{});
		}

		// produce the roots for the different vehicle couplings:
		template <std::size_t N>
		Node<N> produce_root(Node<n_vehicles> const &root, std::set<unsigned int> const &indices) {
			auto produce_root_impl = [root, indices]<std::size_t... I>(std::index_sequence<I...>) -> Node<N> {
				return Node<N>({root.trim(*std::next(indices.begin(), I))...}, {root.x(*std::next(indices.begin(), I))...}, {root.y(*std::next(indices.begin(), I))...}, {root.yaw(*std::next(indices.begin(), I))...});
			};
			return produce_root_impl(std::make_index_sequence<N>{});
		}

		std::set<unsigned int> &contains(std::vector<std::set<unsigned int>> &adjacency_converted, unsigned int current) {
			for (std::set<unsigned int> &j : adjacency_converted) {
				if (j.contains(current)) {
					return j;
				}
			}

			adjacency_converted.push_back({current});

			return adjacency_converted.back();
		}

		std::vector<std::set<unsigned int>> calculate_together(CouplingData const &adjacency_matrix) {
			std::vector<std::set<unsigned int>> adjacency_converted;
			for (unsigned int i = 0; i < adjacency_matrix.size(); ++i) {
				std::set<unsigned int> &adjacency_single = contains(adjacency_converted, i);
				for (unsigned int j = i; j < adjacency_matrix.size(); ++j) {
					if (adjacency_matrix[i][j]) {
						adjacency_single.insert(j);
					}
				}
			}

			return adjacency_converted;
		}

		template <unsigned int N>
		void convert_nodes_to_single(Node<N> const *partial_solution, std::set<unsigned int> const &indices) {
			auto node = solution;
			for (auto k = CentralizedGraphSearch<n_vehicles, scenario_type>::_config->n_hp(); k; --k) {
				for (auto i = 0; auto e : indices) {
					node->trim(e) = partial_solution->trim(i);
					node->x(e) = partial_solution->x(i);
					node->y(e) = partial_solution->y(i);
					node->yaw(e) = partial_solution->yaw(i);
					node->g() += partial_solution->g();
					++i;
				}
				node = node->parent();
				partial_solution = partial_solution->parent();
			}
		}
	};
}  // namespace GraphBasedPlanning