#pragma once

#include <PriorityBasedNode.h>

#include <atomic>
#include <boost/heap/binomial_heap.hpp>

#include "ConfigData.h"
#include "MPA.h"
#include "PriorityBasedGraphSearch.h"
#include "VehicleData.h"

namespace GraphBasedPlanning {
	template <SCENARIO_TYPE scenario_type>
	class GraphSearchPBIncrementalOptimal : public PriorityBasedGraphSearch<scenario_type> {
		PriorityBasedNode *_root = nullptr;
		std::uint64_t n_expanded = 0;
		boost::heap::binomial_heap<PriorityBasedNode *, boost::heap::mutable_<true>, boost::heap::compare<typename PriorityBasedNode::priority_queue_comparison>> heap;
		boost::heap::binomial_heap<PriorityBasedNode *, boost::heap::mutable_<true>, boost::heap::compare<typename PriorityBasedNode::priority_queue_comparison>> blocked_nodes;
		PriorityBasedNode *_solution;
		std::set<PriorityBasedNode *> checked_nodes;
		bool initialized = false;

	   public:
		GraphSearchPBIncrementalOptimal(std::shared_ptr<GraphBasedPlanning::ConfigData const> const &config, std::shared_ptr<GraphBasedPlanning::MPA const> const &mpa, std::array<VehicleData<scenario_type>, 1> const &&vehicle_data,
		    VehicleObstaclesData const &&vehicle_obstacles_data)
		    : PriorityBasedGraphSearch<scenario_type>(config, mpa, std::forward<decltype(vehicle_data)>(vehicle_data), std::forward<decltype(vehicle_obstacles_data)>(vehicle_obstacles_data)) {}

		std::atomic<bool> preperation_done = true;
		std::atomic_flag prep_thread_lock = ATOMIC_FLAG_INIT;
		typedef typename boost::heap::binomial_heap<PriorityBasedNode *, boost::heap::mutable_<true>, boost::heap::compare<PriorityBasedNode::priority_queue_comparison>>::handle_type heap_handle_t;

		PriorityBasedNode const *search(PriorityBasedNode const root) final {
			if (!initialized) [[unlikely]] {
				// Node<FLOATING_POINT_TYPE>::init_static_variables_pb();
				//  set active node heap to vehicle's heap
				PriorityBasedNode::heap() = &heap;
				initialized = true;
			}

			n_expanded = 0;
			checked_nodes.clear();

			if (_root == nullptr) [[unlikely]] {
				// init incremental search
				_root = new PriorityBasedNode(root);
				n_expanded++;
				expand_node(_root, [this](PriorityBasedNode *node) -> heap_handle_t { return heap.push(node); });

				Printer::println("new");
			} else {
				if (_root->x(0) == root.x(0) && _root->y(0) == root.y(0)) {
					apply_function_recursivly(_root, [this](PriorityBasedNode *node) {
						unsigned int i = 0;
						unsigned int j = this->_config->n_hp() - 1;
						double max_distance_traveled = this->_config->dt_seconds() * this->_vehicle_data[i].v_ref(j) * (this->_config->n_hp() - node->k());
						double norm_x_h = node->x(i) - this->_vehicle_data[i].reference_trajectory_point(j).x;  //_reference_trajectory_points(i, j, 0);
						double norm_y_h = node->y(i) - this->_vehicle_data[i].reference_trajectory_point(j).y;
						double norm_squared_h = std::pow(std::max<>(std::sqrt(norm_x_h * norm_x_h + norm_y_h * norm_y_h) - max_distance_traveled, 0.0), 2);

						node->h() += norm_squared_h;

						if (node->heap_handle().has_value()) {
							node->heap()->increase(node->heap_handle().value());
						}
					});
					Printer::println("old");
				} else {
					// reset inc search
					// TODO Future Work: Implement rollback instead of reset
					_root->reset_k_offset();
					delete _root;
					assert(heap.empty());

					// init incremental search
					_root = new PriorityBasedNode(root);
					n_expanded++;
					expand_node(_root, [this](PriorityBasedNode *node) -> heap_handle_t { return heap.push(node); });
					Printer::println("reset");
				}
			}

			while (true) {
				if (!heap.empty()) {
					PriorityBasedNode *const current_node = heap.top();
					heap.pop();
					current_node->reset_heap_handle();

					this->set_predicted_areas(current_node);

					if (!eval_path(current_node)) {
						continue;
					}

					if (this->is_target_reached(current_node)) [[unlikely]] {
						// push node to heap again, because it is not yet expanded but relevant for next iteration
						current_node->set_heap_handle(heap.push(current_node));
						// only temp value. Will be set correctly during prep of next step
						_solution = current_node;
						return current_node;
					}

					expand_node(current_node, [this](PriorityBasedNode *node) -> heap_handle_t { return heap.push(node); });

				}
				// graph search exhausted
				else {
					return nullptr;
				}
			}
		}

		/*[[nodiscard]] PriorityBasedNode<n_vehicles> const *find_solution(ColMajorMatrixAccessor<FLOATING_POINT_TYPE> const &x0, std::vector<std::uint8_t> const &trim_indices) final {
		    if (root == nullptr) [[unlikely]] {
		        init_incremental_search(x0, trim_indices);
		    } else {
		        if (x0(0, 0) == root->x(0) && x0(0, 1) == root->y(0)) {
		            apply_function_recursivly(root, [this](Node<FLOATING_POINT_TYPE> *node) {
		                unsigned int i = 0;
		                unsigned int j = n_hp() - 1;
		                FLOATING_POINT_TYPE max_distance_traveled = dt() * _v_ref(i, j) * (n_hp() - node->k());
		                FLOATING_POINT_TYPE norm_x_h = node->x(i) - _reference_trajectory_points(i, j, 0);
		                FLOATING_POINT_TYPE norm_y_h = node->y(i) - _reference_trajectory_points(i, j, 1);
		                FLOATING_POINT_TYPE norm_squared_h = std::pow(std::max(std::sqrt(norm_x_h * norm_x_h + norm_y_h * norm_y_h) - max_distance_traveled, 0.0), 2);

		                node->h() += norm_squared_h;

		                if (node->heap_handle().has_value()) {
		                    node->heap()->increase(node->heap_handle().value());
		                }
		            });
		        } else {
		            // reset inc search
		            // TODO Future Work: Implement rollback instead of reset
		            root->reset_k_offset();
		            delete root;
		            assert(heap.empty());
		            init_incremental_search(x0, trim_indices);
		        }
		    }

		    while (true) {
		        if (!heap.empty()) {
		            Node<FLOATING_POINT_TYPE> *const current_node = heap.top();
		            heap.pop();
		            current_node->reset_heap_handle();

		            set_predicted_areas(current_node);

		            if (!eval_path(current_node)) {
		                continue;
		            }

		            if (is_target_reached(current_node)) [[unlikely]] {
		                // push node to heap again, because it is not yet expanded but relevant for next iteration
		                current_node->set_heap_handle(heap.push(current_node));
		                // only temp value. Will be set correctly during prep of next step
		                _solution = current_node;
		                return current_node;
		            }

		            expand_node(current_node, [this](Node<FLOATING_POINT_TYPE> *node) -> heap_handle_t { return heap.push(node); });
		        }
		        // graph search exhausted
		        else {
		            return nullptr;
		        }
		    }
		}*/

		void prepare_next_iteration() {
			while (prep_thread_lock.test_and_set(std::memory_order_acquire))
				;
			// merge heaps. All blocked nodes are considered during next iteration
			heap.merge(blocked_nodes);

			if (_solution->k() == 1 || _solution == nullptr) {
				// probably many exhausted graph searches in row. No remaining nodes. reset incremental search
				_root->reset_k_offset();
				delete _root;
				_root = nullptr;
				preperation_done.store(true);
				prep_thread_lock.clear(std::memory_order_release);
				return;
			}
			// set subtree as new tree
			PriorityBasedNode *_next_node = _solution;
			while (_next_node->k() != 1) {
				_next_node = static_cast<PriorityBasedNode *>(_next_node->parent());
			}
			for (unsigned int i = 0; i < _root->children_size(); ++i) {
				if (_root->child(i) == _next_node) {
					_root->child(i) = nullptr;
					break;
				}
			}
			delete _root;
			_root = _next_node;
			_root->increase_k_offset();

			// decrease g
			double g_offset = _root->g();

			apply_function_recursivly(_root, [&g_offset](PriorityBasedNode *node) {
				node->g() -= g_offset;
				if (node->heap_handle().has_value()) {
					node->heap()->decrease(node->heap_handle().value());
				}
			});

			preperation_done.store(true);
			prep_thread_lock.clear(std::memory_order_release);
		}

		void apply_function_recursivly(PriorityBasedNode *node, const std::function<void(PriorityBasedNode *)> &modify_node) {
			for (std::size_t i = 0; i < node->children_size(); ++i) {
				apply_function_recursivly(node->child(i), modify_node);
			}
			modify_node(node);
		}

		void apply_function_to_leaves(PriorityBasedNode *node, const std::function<void(PriorityBasedNode *)> &modify_leaf) {
			for (std::size_t i = 0; i < node->children_size(); ++i) {
				apply_function_to_leaves(node->child(i), modify_leaf);
			}
			if (node->children_size() == 0) {
				modify_leaf(node);
			}
		}

		void update_vehicle_data(std::array<VehicleData<scenario_type>, 1> vehicle_data) { this->_vehicle_data = vehicle_data; }

		void clean() final {
			// prepare for next step in extra thread
			prep_thread_lock.clear(std::memory_order_release);
			std::thread prep_thread(&GraphSearchPBIncrementalOptimal::prepare_next_iteration, this);
			prep_thread.detach();
		}
		~GraphSearchPBIncrementalOptimal() { delete _root; }

		uint64_t get_n_expanded() { return n_expanded; }

	   private:
		void expand_node(PriorityBasedNode *node, const std::function<heap_handle_t(PriorityBasedNode *)> &push) {
			std::array<std::uint8_t, 1> const sizes = this->_mpa->template get_reachable_states_sizes<1>(node->trims(), node->k());
			std::array<unsigned int, 1> const multipliers = this->_mpa->template get_reachable_states_multipliers<1>(sizes);

			auto const n_children = multipliers.back() * sizes.back();
			n_expanded += n_children;

			node->create_children(n_children);

			// use transition matrix from 1st step to expand all nodes. Validity is checked in eval_edge()
			for (auto i = 0; i < n_children; ++i) {
				auto const next_states = this->_mpa->template reachable_states<1>(multipliers, i, node->trims(), sizes, node->k());

				auto *new_node = new PriorityBasedNode(node, node->g(), node->k() + 1);
				this->_mpa->template init_node<1>(new_node, next_states);
				this->init_cost(new_node);

				node->child(i) = new_node;
				new_node->set_heap_handle(push(new_node));
			}
		}

		[[nodiscard]] bool eval_path(PriorityBasedNode *const node) {
			// eval_edge start at root. If node is in the ckecked node set, skip node and continue
			// when reaching node. Assume given node is not yet expanded --> perform lanelet checking and check trim
			// As soon as eval_edge returns true --> push node to checked set
			// As soon as eval_edge returns false --> move all its leaf nodes to blocked heap
			std::vector<PriorityBasedNode *> path;
			path.reserve(node->k() - 1);
			for (PriorityBasedNode *node_i = static_cast<PriorityBasedNode *>(node->parent()); node_i != _root; node_i = static_cast<PriorityBasedNode *>(node->parent())) {
				path.push_back(node_i);
			}
			for (auto it = path.rbegin(); it != path.rend(); ++it) {
				if (!checked_nodes.contains(*it)) {
					if (!eval_edge(*it, false, false, false)) {
						// move all node leaves(=non-expanded) of blocking node to blocked node heap
						// This will also push the node for which eval_path was called to the blocked node heap
						apply_function_to_leaves(*it, [this](PriorityBasedNode *node) {
							// remove from regular heap if still on the heap
							if (node->heap_handle().has_value()) {
								node->heap()->erase(node->heap_handle().value());
							}
							node->set_heap_handle(blocked_nodes.push(node));
						});
						return false;
					} else {
						checked_nodes.insert(*it);
					}
				}
			}
			// Finally, check the node for which eval_path was called. This node shouldn't be expanded yet. Thus, check also lanelet boundaries and trim.
			if (!eval_edge(node, true, true, true)) {
				return false;
			}
			checked_nodes.insert(node);
			return true;
		}

		[[nodiscard]] bool eval_edge(PriorityBasedNode *const node, bool check_trim, bool check_lanelet, bool push_node) {
			// check if trim is valid at current time step. (related to recursive feasibility)
			if (check_trim && this->_config->recursive_feasibility() && !check_trim_valid(node)) {
				// mark nodes as blocked. Node possibly still relevant for next iteration
				// push only non-expanded nodes to heap
				node->set_heap_handle(blocked_nodes.push(node));
				return false;
			}

			// other vehicle obstacles(including static obstacles) against normal offset
			if (!this->check_vehicle_obstacles(node)) {
				// mark nodes as blocked. Node possibly still relevant for next iteration
				// push only non-expanded nodes to heap
				if (push_node) {
					node->set_heap_handle(blocked_nodes.push(node));
				}
				return false;
			}

			// lanelet interaction:

			if constexpr (scenario_type != SCENARIO_TYPE::Circle) {
				std::array<std::vector<vec2>, 1> vehicles_obstacle;
				vehicles_obstacle = this->_mpa->template calc_vehicles_obstacles_without_offset<1>(node);
				if (check_lanelet && !this->lanelet_interaction_valid(vehicles_obstacle[0], 0)) {
					// collides with lanelet boundaries. Thus, not relevant for this or following iterations.
					// TODO BUG? bad trajectory quality if the following workaround line is removed
					// node->set_heap_handle(blocked_nodes.push(node));
					return false;
				}
			}

			return true;
		}

		[[nodiscard]] bool check_trim_valid(PriorityBasedNode *const node) const {
			std::uint8_t steps_to_go = this->_config->n_hp() - node->k();
			return this->_mpa->distance_to_equilibrium(node->trim(0)) <= steps_to_go;
		}
	};
}  // namespace GraphBasedPlanning