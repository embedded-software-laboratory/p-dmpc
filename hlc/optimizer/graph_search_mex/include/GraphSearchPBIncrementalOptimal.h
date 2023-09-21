#pragma once

#include "GraphSearchSpecialization.h"
#include <atomic>
#include <boost/heap/binomial_heap.hpp>

namespace GraphBasedPlanning {
	class GraphSearchPBIncrementalOptimal : private GraphSearchSpecialization {
		Node<FLOATING_POINT_TYPE> *root = nullptr;
		uint64_t n_expanded = 0;
		boost::heap::binomial_heap<Node<FLOATING_POINT_TYPE> *, boost::heap::mutable_<true>, boost::heap::compare<Node<FLOATING_POINT_TYPE>::priority_queue_comparison>> heap;
		boost::heap::binomial_heap<Node<FLOATING_POINT_TYPE> *, boost::heap::mutable_<true>, boost::heap::compare<Node<FLOATING_POINT_TYPE>::priority_queue_comparison>> blocked_nodes;
		Node<FLOATING_POINT_TYPE>* _solution;
		std::set<Node<FLOATING_POINT_TYPE>*> checked_nodes;
		bool initialized = false;

	   public:
	   	std::atomic<bool> preperation_done = true;
		std::atomic_flag prep_thread_lock = ATOMIC_FLAG_INIT;
		typedef typename boost::heap::binomial_heap<Node<double> *, boost::heap::mutable_<true>, boost::heap::compare<Node<double>::priority_queue_comparison>>::handle_type heap_handle_t;

		[[nodiscard]] Node<FLOATING_POINT_TYPE> const *find_solution(ColMajorMatrixAccessor<FLOATING_POINT_TYPE> const &x0, std::vector<std::uint8_t> const &trim_indices) final {

			if (!initialized) [[unlikely]] {
				//Node<FLOATING_POINT_TYPE>::init_static_variables_pb();
				// set active node heap to vehicle's heap
				Node<FLOATING_POINT_TYPE>::heap() = &heap;
				initialized = true;
			}

			n_expanded = 0;
			checked_nodes.clear();

			if (root == nullptr) [[unlikely]] {
				init_incremental_search(x0, trim_indices);
			}
			else {
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
				if(!heap.empty()) {
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
						//only temp value. Will be set correctly during prep of next step
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
		}

		void prepare_next_iteration() {
			while (prep_thread_lock.test_and_set(std::memory_order_acquire));
			// merge heaps. All blocked nodes are considered during next iteration
			heap.merge(blocked_nodes);

			if (_solution->k() == 1 || _solution == nullptr) {
				// probably many exhausted graph searches in row. No remaining nodes. reset incremental search
				root->reset_k_offset();
				delete root;
				root = nullptr;
				preperation_done.store(true);
				prep_thread_lock.clear(std::memory_order_release);
				return;
			}
			//set subtree as new tree
			Node<FLOATING_POINT_TYPE> *_next_node = _solution;
			while (_next_node->k()!=1)
			{
				_next_node = _next_node->parent();
			}			
			for (unsigned int i = 0; i < root->children_size(); ++i) {
				if (root->children(i) == _next_node) {
					root->set_children(i, nullptr);
					break;
				}		
			}
			delete root;
			root = _next_node;
			root->increase_k_offset();

			//decrease g
			FLOATING_POINT_TYPE g_offset = root->g();

			apply_function_recursivly(root, [&g_offset](Node<FLOATING_POINT_TYPE> *node) { 
				node->g() -= g_offset;
				if (node->heap_handle().has_value()) {
					node->heap()->decrease(node->heap_handle().value());
				}	
			});

			preperation_done.store(true);
			prep_thread_lock.clear(std::memory_order_release);
		}


		void apply_function_recursivly(Node<FLOATING_POINT_TYPE> *node, const std::function<void(Node<FLOATING_POINT_TYPE> *)> &modify_node) {
			for (std::size_t i = 0; i < node->children_size(); ++i) {
				apply_function_recursivly(node->children(i), modify_node);
			}
			modify_node(node);			
		}

		void apply_function_to_leaves(Node<FLOATING_POINT_TYPE> *node, const std::function<void(Node<FLOATING_POINT_TYPE> *)> &modify_leaf) {
			for (std::size_t i = 0; i < node->children_size(); ++i) {
				apply_function_to_leaves(node->children(i), modify_leaf);
			}				
			if (node->children_size() == 0) {
				modify_leaf(node);
			}
		}

		void clean() final { }

		uint64_t get_n_expanded() {return n_expanded; }		

	   private:

		void init_incremental_search(ColMajorMatrixAccessor<FLOATING_POINT_TYPE> const &x0, std::vector<std::uint8_t> const &trim_indices){
			root = Node<FLOATING_POINT_TYPE>::create_root_node(trim_indices.begin(), &x0(0, 0), &x0(0, 1), &x0(0, 2));
			n_expanded++;
			expand_node(root, [this](Node<FLOATING_POINT_TYPE> *node) -> heap_handle_t { return heap.push(node); });
		}

		[[nodiscard]] Node<FLOATING_POINT_TYPE> *create_new_node(Node<FLOATING_POINT_TYPE> *const node, std::uint8_t const *const next_trims, std::uint64_t const next_step) const {
			Node<FLOATING_POINT_TYPE> *const new_node = Node<FLOATING_POINT_TYPE>::create_node(node->g(), node, next_step);

			init_new_node(node, next_trims, new_node);

			return new_node;
		}

		void expand_node(Node<FLOATING_POINT_TYPE> *node, const std::function<heap_handle_t(Node<FLOATING_POINT_TYPE> *)> &push) {
			std::uint8_t const next_step = node->k() + 1U;

			std::uint8_t *sizes = get_reachable_states_sizes(&node->trim(0), next_step);
			unsigned int *multipliers = get_reachable_states_multipliers(sizes);

			node->create_children(multipliers[n_vehicles()]);
			//use transition matrix from 1st step to expand all nodes. Validity is checked in eval_edge()
			for (unsigned int k = 0; uint8_t const *const next_trims : reachable_states(&node->trim(0), sizes, multipliers, 1)) {
				Node<FLOATING_POINT_TYPE> *const new_node = create_new_node(node, next_trims, next_step + node->k_offset());
				n_expanded++;

				node->children(k++) = new_node;
				new_node->set_heap_handle(push(new_node));
			}		
		}

		[[nodiscard]] bool eval_path(Node<FLOATING_POINT_TYPE> *const node) {
			// eval_edge start at root. If node is in the ckecked node set, skip node and continue
			// when reaching node. Assume given node is not yet expanded --> perform lanelet checking and check trim
			// As soon as eval_edge returns true --> push node to checked set
			// As soon as eval_edge returns false --> move all its leaf nodes to blocked heap
			std::vector<Node<FLOATING_POINT_TYPE>*> path;
			path.reserve(node->k() - 1);
			for (Node<FLOATING_POINT_TYPE> *node_i = node->parent(); node_i != root; node_i = node_i->parent()) {
				path.push_back(node_i);
			}
			for (auto it = path.rbegin(); it != path.rend(); ++it) {
				if (!checked_nodes.contains(*it)) {
					if (!eval_edge(*it, false, false, false)) {
						// move all node leaves(=non-expanded) of blocking node to blocked node heap
						// This will also push the node for which eval_path was called to the blocked node heap
						apply_function_to_leaves(*it, [this](Node<FLOATING_POINT_TYPE> *node) {
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

		[[nodiscard]] bool eval_edge(Node<FLOATING_POINT_TYPE> *const node, bool check_trim, bool check_lanelet, bool push_node) {

			//check if trim is valid at current time step. (related to recursive feasibility)
			if (check_trim && recursive_feasibility() && !check_trim_valid(node)) {
				// mark nodes as blocked. Node possibly still relevant for next iteration
				// push only non-expanded nodes to heap
				node->set_heap_handle(blocked_nodes.push(node));
				return false;
			}

			// other vehicle obstacles(including static obstacles) against normal offset
			if (!check_vehicle_obstacles(node)) {
				// mark nodes as blocked. Node possibly still relevant for next iteration
				// push only non-expanded nodes to heap
				if (push_node) {
					node->set_heap_handle(blocked_nodes.push(node));
				}
				return false;
			}

			// lanelet interaction:
			std::vector<std::vector<CollisionDetection::vec2>> vehicles_shapes(n_vehicles());
			get_vehicle_shapes_no_offset(node, vehicles_shapes);
			if (check_lanelet && !check_lanelet_interaction(node, vehicles_shapes)) {
				//collides with lanelet boundaries. Thus, not relevant for this or following iterations.
				//TODO BUG? bad trajectory quality if the following workaround line is removed
				//node->set_heap_handle(blocked_nodes.push(node));
				return false;
			}
			return true;
		}

		[[nodiscard]] bool check_trim_valid(Node<FLOATING_POINT_TYPE> *const node) const {
			uint8_t steps_to_go = n_hp() - node->k();
			return _distance_to_equilibrium[node->trim(0)] <= steps_to_go;
		}

		void get_vehicle_shapes_no_offset(Node<FLOATING_POINT_TYPE> *const node, std::vector<std::vector<CollisionDetection::vec2>> &vehicles_obstacles) const {
			for (unsigned int i = 0; i < n_vehicles(); ++i) {
				auto c = std::cos(node->parent()->yaw(i));
				auto s = std::sin(node->parent()->yaw(i));

			auto const &points = area_without_offset(node->parent()->trim(i), node->trim(i));
			vehicles_obstacles[i].resize(points.size() / 2);

			for (unsigned int j = 0; j < vehicles_obstacles[i].size(); ++j) {
				vehicles_obstacles[i][j].x = c * points[j * 2] - s * points[j * 2 + 1] + node->parent()->x(i);
				vehicles_obstacles[i][j].y = s * points[j * 2] + c * points[j * 2 + 1] + node->parent()->y(i);
			}
			}
		}
	};
}  // namespace GraphBasedPlanning