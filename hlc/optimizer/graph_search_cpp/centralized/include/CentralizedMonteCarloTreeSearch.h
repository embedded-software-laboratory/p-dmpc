#pragma once

#include <MatlabException.h>
#include <MonteCarloTreeSearchNode.h>
#include <Node.h>
#include <Printer.h>
#include <RandomSimulationNode.h>

#include <boost/heap/priority_queue.hpp>
#include <numbers>
#include <queue>
#include <random>

#include "CentralizedGraphSearch.h"
#include "ConfigData.h"
#include "MPA.h"
#include "Sampler.h"
#include "VehicleData.h"

namespace GraphBasedPlanning {
	template <unsigned int n_vehicles, SCENARIO_TYPE scenario_type>
	class CentralizedMonteCarloTreeSearch : public CentralizedGraphSearch<n_vehicles, scenario_type>, private Sampler<> {
		Node<n_vehicles> *_min;
		MonteCarloTreeSearchNode<n_vehicles> *_root = nullptr;
		std::vector<double> _normalization_values;
		unsigned int _minima_updates = 0;
		unsigned int _updates = 0;
		unsigned int _random_minima_updates = 0;
		unsigned int _random_updates = 0;

	   public:
		CentralizedMonteCarloTreeSearch(std::shared_ptr<GraphBasedPlanning::ConfigData const> const &config, std::shared_ptr<GraphBasedPlanning::MPA const> const &mpa, std::array<VehicleData<scenario_type>, n_vehicles> const &&vehicle_data)
		    : CentralizedGraphSearch<n_vehicles, scenario_type>(config, mpa, std::forward<decltype(vehicle_data)>(vehicle_data)), _normalization_values(CentralizedGraphSearch<n_vehicles, scenario_type>::_config->n_hp()) {
			_min = new MonteCarloTreeSearchNode<n_vehicles>(nullptr, std::numeric_limits<double>::infinity(), CentralizedGraphSearch<n_vehicles, scenario_type>::_config->n_hp(), (std::numeric_limits<unsigned int>::max)());

			// calculate the normalization values for rating the cost to come
			for (auto i = 0; i < CentralizedGraphSearch<n_vehicles, scenario_type>::_config->n_hp(); ++i) {
				for (auto j = 0; j < n_vehicles; ++j) {
					_normalization_values[i] += CentralizedGraphSearch<n_vehicles, scenario_type>::_vehicle_data[j].v_ref(i) * CentralizedGraphSearch<n_vehicles, scenario_type>::_config->dt();
				}
			}

			for (auto i = 1; i < CentralizedGraphSearch<n_vehicles, scenario_type>::_config->n_hp(); ++i) {
				_normalization_values[i] += _normalization_values[i - 1] * 2;
			}
		}
		Node<n_vehicles> const *search(Node<n_vehicles> const root) final {
			_root = new MonteCarloTreeSearchNode<n_vehicles>(root);
			unsigned int const n_shots = 100000;

			std::array<std::uint8_t, n_vehicles> const sizes = CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->template get_reachable_states_sizes<n_vehicles>(_root->trims(), _root->k());
			std::array<unsigned int, n_vehicles> const multipliers = CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->template get_reachable_states_multipliers<n_vehicles>(sizes);

			auto const n_children = multipliers[n_vehicles - 1] * sizes[n_vehicles - 1];
			_root->create_children(n_children);

			double N = 0.0;
			auto root_ucb1_index = [this, &N]() {
				static double const c = std::sqrt(2);

				auto max_index = -1;
				double max = -std::numeric_limits<double>::infinity();

				for (auto i = 0; i < _root->children_size(); ++i) {
					double const ucb1 = _root->w(i) / (_root->n(i) + 1.0) + c * std::sqrt(std::log(N + 1.0) / (_root->n(i) + 1.0));
					if (ucb1 > max) {
						max = ucb1;
						max_index = i;
					}
				}

				return max_index;
			};

			for (auto i = 0; i < n_shots; ++i) {
				// find index
				auto index = root_ucb1_index();

				if (!_root->child(index)) {  // init node at index
					MonteCarloTreeSearchNode<n_vehicles> *new_node = new MonteCarloTreeSearchNode<n_vehicles>(_root, _root->g(), _root->k() + 1, index);
					auto const next_states = CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->template reachable_states<n_vehicles>(multipliers, index, _root->trims(), sizes, _root->k());

					CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->template init_node<n_vehicles>(new_node, next_states);
					init_cost(new_node);

					// check collision of new node and deactivate and delete if necessary
					std::array<std::vector<vec2>, n_vehicles> const vehicles_obstacles = CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->template calc_vehicles_obstacles_without_offset<n_vehicles>(new_node);
					if (!CentralizedGraphSearch<n_vehicles, scenario_type>::is_path_valid(vehicles_obstacles)) {
						_root->deactivate_child(index);  // doesn't get chosen by ucb1 anymore, hopefully
						delete new_node;
						continue;
					}

					_root->child(index) = new_node;
				}

				// add ucb1 values to child
				_root->w(index) += random_expand_node(_root->child(index));
				_root->n(index) += 1.0;
				N += 1.0;
			}

			if (_min->g() == std::numeric_limits<double>::infinity()) {
				throw MatlabException("NO Solution found!");
			}

			return _min;
		}
		void clean() final {
			Printer::println(_minima_updates);
			Printer::println(_updates);
			Printer::println(_random_minima_updates);
			Printer::println(_random_updates);
			delete this;  // commit suicide
		}

		~CentralizedMonteCarloTreeSearch() { delete _root; }

	   private:
		void print_children(double N) {
			Printer::println(N);

			// for (auto i = 0; i < _root->children_size(); ++i) {
			//	Printer::print(_root->w(i), "/", _root->n(i), ", ");
			// }
			// Printer::println();

			// for (auto i = 0; i < _root->child(0)->children_size(); ++i) {
			//	Printer::print(_root->child(0)->w(i), "/", _root->child(0)->n(i), ", ");
			// }
			// Printer::println();
		}

		bool random_expand_node(MonteCarloTreeSearchNode<n_vehicles> *const node) {
			if (node->children_size()) {  // node has already done a random simulation
				// find index
				auto index = node->ucb1_index();

				if (!node->child(index)) {  // init node at index
					std::array<std::uint8_t, n_vehicles> const sizes = CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->template get_reachable_states_sizes<n_vehicles>(node->trims(), node->k());
					std::array<unsigned int, n_vehicles> const multipliers = CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->template get_reachable_states_multipliers<n_vehicles>(sizes);

					MonteCarloTreeSearchNode<n_vehicles> *new_node = new MonteCarloTreeSearchNode<n_vehicles>(node, node->g(), node->k() + 1, index);
					auto const next_states = CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->template reachable_states<n_vehicles>(multipliers, index, node->trims(), sizes, node->k());

					CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->template init_node<n_vehicles>(new_node, next_states);
					init_cost(new_node);

					// check collision of new node and deactivate and delete if necessary
					std::array<std::vector<vec2>, n_vehicles> const vehicles_obstacles = CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->template calc_vehicles_obstacles_without_offset<n_vehicles>(new_node);
					if (!CentralizedGraphSearch<n_vehicles, scenario_type>::is_path_valid(vehicles_obstacles)) {
						node->deactivate_child(index);  // doesn't get chosen by ucb1 anymore, hopefully
						delete new_node;
						return false;  // a loss...
					}

					node->child(index) = new_node;
				}

				// add ucb1 values to child
				bool ret = false;
				if (index > 0) ret = random_expand_node(node->child(index));
				node->w(index) += ret;
				node->n(index) += 1.0;
				return ret;
			}

			if (node->k() < CentralizedGraphSearch<n_vehicles, scenario_type>::_config->n_hp()) {
				std::array<std::uint8_t, n_vehicles> const sizes = CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->template get_reachable_states_sizes<n_vehicles>(node->trims(), node->k());
				std::array<unsigned int, n_vehicles> const multipliers = CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->template get_reachable_states_multipliers<n_vehicles>(sizes);

				auto const n_children = multipliers[n_vehicles - 1] * sizes[n_vehicles - 1];
				node->create_children(n_children);

				return random_simulation(node);
			} else {
				bool ret = node->g() < _min->g();
				if (ret) {
					delete _min;
					_min = node;

					++_minima_updates;
				}

				++_updates;
				static_cast<MonteCarloTreeSearchNode<n_vehicles> *const>(node->parent())->deactivate_child(node->index());  // because end node

				return ret;
			}

			/*
			if (index < 0.0) return;

			if (node->child(index)) {
			    random_expand_node(node->child(index));
			} else {
			    MonteCarloTreeSearchNode<n_vehicles> *new_node = new MonteCarloTreeSearchNode<n_vehicles>(node, node->g(), node->k() + 1, index);
			    auto const next_states = CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->template reachable_states<n_vehicles>(multipliers, index, node->trims(), sizes, node->k());

			    CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->template init_node<n_vehicles>(new_node, next_states);
			    init_cost(new_node);

			    node->child(index) = new_node;

			    // test if new child causes a collision:
			    if (new_node->k() < CentralizedGraphSearch<n_vehicles, scenario_type>::_config->n_hp()) {
			        std::array<std::vector<vec2>, n_vehicles> const vehicles_obstacles = CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->template calc_vehicles_obstacles_without_offset<n_vehicles>(new_node);
			        if (!CentralizedGraphSearch<n_vehicles, scenario_type>::is_path_valid(vehicles_obstacles)) {
			            node->probability(index) = 0.0;
			            // Printer::println("lol");

			            // TODO: maybe change parents' probability values, too.
			            return;  // node is useless
			        }
			    } else {
			        if (new_node->g() < _min->g()) {
			            std::array<std::vector<vec2>, n_vehicles> const vehicles_obstacles = CentralizedGraphSearch<n_vehicles, scenario_type>::_mpa->template calc_vehicles_obstacles_without_offset<n_vehicles>(new_node);
			            if (CentralizedGraphSearch<n_vehicles, scenario_type>::is_path_valid(vehicles_obstacles)) {
			                _min = new_node;
			                ++_minima_updates;

			                // MonteCarloTreeSearchNode<n_vehicles> *tmp_parent = static_cast<MonteCarloTreeSearchNode<n_vehicles> *>(node->parent());
			                // MonteCarloTreeSearchNode<n_vehicles> *tmp_child = node;
			                //
			                // for (auto i = 0; i < CentralizedGraphSearchSpecialization<n_vehicles, scenario_type>::_config->n_hp() - 1; ++i) {
			                //	tmp_parent->probability(tmp_child->index()) *= 2;
			                //	tmp_child = tmp_parent;
			                //	tmp_parent = static_cast<MonteCarloTreeSearchNode<n_vehicles> *>(tmp_parent->parent());
			                //}
			            }
			        }

			        ++_updates;

			        node->probability(index) = 0.0;
			        return;
			    }

			    // Simulation: + // Backpropagation:
			    double g = random_simulation(new_node);

			    if (g >= 0.0) {
			        // normalizing g:
			        // linear rating: max distance: 2 * v_ref*dt, min distance: 0
			        // auto const rating = std::max(-2.0 * g / _normalization_values.back() + 2.0, 0.0);

			        auto const rating = -g / _normalization_values.back() + 1.0;
			        auto const probability = std::exp(rating);

			        for (auto i = new_node->k(); i; --i) {
			            static_cast<MonteCarloTreeSearchNode<n_vehicles> *const>(new_node->parent())->probability(new_node->index()) += probability * new_node->k() * new_node->k() * new_node->k() * n_vehicles * n_vehicles * n_vehicles;
			            new_node = static_cast<MonteCarloTreeSearchNode<n_vehicles> *const>(new_node->parent());
			        }

			        ////auto const rating = -2.0 * g / _normalization_values.back() + 2.0;
			        ////if (rating >= 0.0) {
			        ////	auto const probability = std::log(rating + 1.0) + 1.0;
			        ////	for (auto i = new_node->k(); i; --i) {
			        ////		static_cast<MonteCarloTreeSearchNode<n_vehicles> *const>(new_node->parent())->probability(new_node->index()) += probability;
			        ////		//Printer::println(new_node->k(), ", ", rating, ", ", probability, ",", static_cast<MonteCarloTreeSearchNode<n_vehicles> *const>(new_node->parent())->probability(new_node->index()));
			        ////		new_node = static_cast<MonteCarloTreeSearchNode<n_vehicles> *const>(new_node->parent());
			        ////	}
			        ////} else {
			        ////	for (auto i = new_node->k(); i; --i) {
			        ////		Printer::println(static_cast<MonteCarloTreeSearchNode<n_vehicles> *const>(new_node->parent())->probability(new_node->index()));
			        ////		static_cast<MonteCarloTreeSearchNode<n_vehicles> *const>(new_node->parent())->probability(new_node->index()) =
			        ////		    std::pow(static_cast<MonteCarloTreeSearchNode<n_vehicles> *const>(new_node->parent())->probability(new_node->index()), rating);
			        ////		new_node = static_cast<MonteCarloTreeSearchNode<n_vehicles> *const>(new_node->parent());
			        ////	}
			        ////}

			        // Printer::println("root: ");
			        // for(auto i = 0; i < _root->children_size(); ++i) {
			        //	Printer::print(_root->probability(i), ", ");
			        // }
			        // Printer::println();

			        // auto const probability = 4.0 / (std::numbers::e * std::numbers::e * std::numbers::e * std::numbers::e) * std::exp(rating) * std::exp(rating);

			        // Printer::println(rating);

			        // static auto const cot1 = 1.0 / tan(1.0);
			        // auto const probability = cot1 * std::tan(rating - 1.0) + 1.0;
			        //  Printer::println(node->k(), ", ", g, ", ", rating, ", ", probability);
			        // node->probability(new_node->index()) *= probability;

			        // node->probability(new_node->index()) += new_node->k() * rating;
			        // for (auto i = new_node->k(); i; --i) {
			        //	//Printer::println(static_cast<MonteCarloTreeSearchNode<n_vehicles> *const>(new_node->parent())->probability(new_node->index()));
			        //	static_cast<MonteCarloTreeSearchNode<n_vehicles> *const>(new_node->parent())->probability(new_node->index()) += new_node->k() * rating;
			        //	new_node = static_cast<MonteCarloTreeSearchNode<n_vehicles> *const>(new_node->parent());
			        // }

			        // if (node->k() > 0) {
			        //	//static auto const cot1_3 = cot1 * cot1 * cot1;
			        //	//auto const probability2 = cot1_3 * std::tan(rating - 1.0) * std::tan(rating - 1.0) * std::tan(rating - 1.0) + 1.0;
			        //	MonteCarloTreeSearchNode<n_vehicles> *const second_node = static_cast<MonteCarloTreeSearchNode<n_vehicles> *const>(node->parent());
			        //	second_node->probability(node->index()) += node->k() * rating;
			        // }

			        // rating: map 0 to *2 and  to *1
			        ////auto probability = 2 * exp(-g * std::numbers::ln2); std::tanh(rating - 1.0) + 1.0;
			        ////
			        ////if (probability == std::numeric_limits<double>::infinity()) Printer::println(g);
			        ////
			        ////node->probability(new_node->index()) *= probability;
			        ////if (node->k() > 0) {
			        ////	MonteCarloTreeSearchNode<n_vehicles> *const second_node = static_cast<MonteCarloTreeSearchNode<n_vehicles> *const>(node->parent());
			        ////
			        ////	auto probability2 = 0.075 * std::tanh(probability - 1.0) + 1.0;
			        ////	static_cast<MonteCarloTreeSearchNode<n_vehicles> *const>(node->parent())->probability(node->index()) *= probability2;
			        ////}
			        ////
			        ////if (node->probability(new_node->index()) == std::numeric_limits<double>::infinity()) {
			        ////	Printer::println(new_node->k(), ", ", g, ", ", probability);
			        ////	throw MexException("INF");
			        ////}

			        // for (auto i = new_node->k(); i; --i) {
			        //	Printer::println(static_cast<MonteCarloTreeSearchNode<n_vehicles> *const>(new_node->parent())->probability(new_node->index()));
			        //	static_cast<MonteCarloTreeSearchNode<n_vehicles> *const>(new_node->parent())->probability(new_node->index()) *= probability;
			        //
			        //	if (static_cast<MonteCarloTreeSearchNode<n_vehicles> *const>(new_node->parent())->probability(new_node->index()) == std::numeric_limits<double>::infinity()) {
			        //		Printer::println(new_node->k(), ", ", g, ", ", probability);
			        //		throw MexException("INF");
			        //	}
			        //
			        //	new_node = static_cast<MonteCarloTreeSearchNode<n_vehicles> *const>(new_node->parent());
			        //}
			    }
			}*/
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

				new_node->g() += std::sqrt(norm_squared_g);
			}
		}

		bool random_simulation(Node<n_vehicles> *node) {
			thread_local static std::random_device rd;
			thread_local static std::mt19937 random{rd()};

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
				return false;
			} else {
				// replace old min value
				if (node->g() < _min->g()) {
					delete _min;
					_min = node;

					++_random_updates;
					++_random_minima_updates;

					return true;
				}

				if (0.3 * node->g() < _min->g()) {
					delete node;

					++_random_updates;

					return true;
				} else {
					delete node;

					return false;
				}
			}

			// double ret = valid ? node->g() : std::numeric_limits<double>::infinity();

			// for (auto i = CentralizedGraphSearch<n_vehicles, scenario_type>::_config->n_hp(); i > old_k; --i) {
			//	auto tmp = node->parent();
			//	delete node;
			//	node = tmp;
			// }

			// return ret;
		}
	};
}  // namespace GraphBasedPlanning