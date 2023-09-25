#pragma once

#include <ChildrenNode.h>
#include <MonteCarloNode.h>
#include <Sampler.h>
#include <Utils.h>
#include <matplotlibcpp.h>

#include <boost/heap/priority_queue.hpp>
#include <map>
#include <memory_resource>
#include <queue>
#include <random>

#include "CentralizedGraphSearch.h"
#include "ConfigData.h"
#include "MPA.h"
#include "VehicleData.h"

namespace GraphBasedPlanning {
	template <unsigned int n_vehicles, SCENARIO_TYPE scenario_type>
	class CentralizedCreatingDistribution : public CentralizedGraphSearch<n_vehicles, scenario_type>, private Sampler<> {
		std::vector<ChildrenNode<n_vehicles> *> _nodes;
		ChildrenNode<n_vehicles> *_root = nullptr;

	   public:
		CentralizedCreatingDistribution(std::shared_ptr<GraphBasedPlanning::ConfigData const> const &config, std::shared_ptr<GraphBasedPlanning::MPA const> const &mpa, std::array<VehicleData<scenario_type>, n_vehicles> const &&vehicle_data)
		    : CentralizedGraphSearch<n_vehicles, scenario_type>(config, mpa, std::forward<decltype(vehicle_data)>(vehicle_data)) {}

		void bfs(ChildrenNode<n_vehicles> *node) {
			if (node->k() == CentralizedGraphSearch<n_vehicles, scenario_type>::_config->n_hp()) {
				_nodes.push_back(node);
				return;
			}

			expand_node(node);
			for (auto i = 0; i < node->children_size(); ++i) {
				bfs(node->baby(i));
			}
		}

		void expand_node(ChildrenNode<n_vehicles> *const node) {
			std::array<std::uint8_t, n_vehicles> const sizes = CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->template get_reachable_states_sizes<n_vehicles>(node->trims(), node->k());
			std::array<unsigned int, n_vehicles> const multipliers = CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->template get_reachable_states_multipliers<n_vehicles>(sizes);

			auto const n_children = multipliers[n_vehicles - 1] * sizes[n_vehicles - 1];

			node->create_children(n_children);

			for (auto i = 0; i < n_children; ++i) {
				auto const next_states = CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->template reachable_states<n_vehicles>(multipliers, i, node->trims(), sizes, node->k());

				auto *new_node = new ChildrenNode<n_vehicles>(node, node->g(), node->k() + 1);
				CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->template init_node<n_vehicles>(new_node, next_states);
				init_cost(new_node);

				node->child(i) = new_node;
			}
		}

		Node<n_vehicles> const *search(Node<n_vehicles> const root) final {
			namespace plt = matplotlibcpp;
			_root = new ChildrenNode<n_vehicles>(root);

			bfs(_root);

			std::vector<double> gs(_nodes.size());
			std::vector<double> points(_nodes.size());
			for (auto i = 0; i < _nodes.size(); ++i) {
				gs[i] = _nodes[i]->g();
				points[i] = i;
			}

			plt::scatter(points, gs);
			plt::show();

			return *std::min_element(_nodes.begin(), _nodes.end(), [](ChildrenNode<n_vehicles> const *const lhs, ChildrenNode<n_vehicles> const *const rhs) { return lhs->g() < rhs->g(); });
		}
		void clean() final {
			delete this;  // commit suicide
		}

		~CentralizedCreatingDistribution() = default;

	   private:
		std::uint64_t plot_index(Node<n_vehicles> const *node, std::integral auto const index) {
			if (node->k() > 1) {
				return plot_index(node->parent(), index) +
				       node->trim(index) * Utils::powi(CentralizedGraphSearch<n_vehicles, scenario_type>::_config->n_trims(), CentralizedGraphSearch<n_vehicles, scenario_type>::_config->n_hp() - node->k());
			} else {
				return node->trim(index) * Utils::powi(CentralizedGraphSearch<n_vehicles, scenario_type>::_config->n_trims(), CentralizedGraphSearch<n_vehicles, scenario_type>::_config->n_hp() - node->k());
			}
		}

		void generate_trim_sequence(std::back_insert_iterator<std::vector<std::uint8_t>> it, Node<n_vehicles> const *node, std::integral auto const index) {
			if (node->k() > 1) {
				generate_trim_sequence(it, node->parent(), index);
				it = node->trim(index);
			} else {
				it = node->trim(index);
			}
		}
		std::vector<std::uint8_t> generate_trim_sequence(Node<n_vehicles> const *node, std::integral auto const index) {
			std::vector<std::uint8_t> res;
			res.reserve(CentralizedGraphSearch<n_vehicles, scenario_type>::_config->n_hp());
			generate_trim_sequence(std::back_insert_iterator<std::vector<std::uint8_t>>(res), node, index);
			return res;
		}

		void plot_distribution() {
			namespace plt = matplotlibcpp;
			plt::figure_size(1200, 780);

			auto n_points = Utils::powi(CentralizedGraphSearch<n_vehicles, scenario_type>::_config->n_trims(), CentralizedGraphSearch<n_vehicles, scenario_type>::_config->n_hp());

			// std::vector<double> points(n_points);
			// std::iota(points.begin(), points.end(), 0.0);
			// std::vector<double> gs(n_points, 0.0);

			// std::vector<double> gs;
			// gs.reserve(_pq.size());
			// std::vector<double> points;
			// gs.reserve(_pq.size());
			//
			// std::vector<std::vector<std::pair<std::vector<std::uint8_t>, double>>> test(n_points);
			//
			// for (auto e : _pq) {
			//	auto const index = plot_index(e, 0);
			//	test[index].push_back({generate_trim_sequence(e, 0), e->g()});
			//
			//	gs.push_back(e->g());
			//	points.push_back(index);
			//}
			//
			// plt::scatter(points, gs);
			// plt::show();
		}

		bool check_path(MonteCarloNode<n_vehicles> const *node) const {
			if (!CentralizedGraphSearch<n_vehicles, scenario_type>::is_path_valid(CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->calc_vehicles_obstacles_large_offset(node))) {
				return false;
			}

			for (auto i = CentralizedGraphSearch<n_vehicles, scenario_type>::_config->n_hp() - 2; i; --i) {  // without root, because vehicle is already on the position; begin at last hp, because highest chance of collision
				node = static_cast<decltype(node)>(node->parent());

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

		// void random_expand(MonteCarloNode<n_vehicles> *node) {
		//	static auto random = std::mt19937{std::random_device{}()};
		//
		//	std::array<std::uint8_t, n_vehicles> next_trims;
		//
		//	for (auto i = 1; i <= CentralizedGraphSearch<n_vehicles, scenario_type>::_config->n_hp(); ++i) {
		//		for (unsigned int j = 0; j < n_vehicles; ++j) {
		//			std::vector<std::uint8_t> const &sampling_vector = CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->reachability_list(i - 1, node->trim(j));
		//			auto const index = sample(sampling_vector.size());
		//			next_trims[j] = sampling_vector[index];
		//		}
		//
		//		MonteCarloNode<n_vehicles> *const new_node = _pa.new_object<MonteCarloNode<n_vehicles>>(node, node->g(), i);
		//		CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->template init_node<n_vehicles>(new_node, next_trims);
		//		init_cost(new_node);
		//
		//		node = new_node;
		//	}
		//
		//	_pq.push(node);
		//}
	};
}  // namespace GraphBasedPlanning