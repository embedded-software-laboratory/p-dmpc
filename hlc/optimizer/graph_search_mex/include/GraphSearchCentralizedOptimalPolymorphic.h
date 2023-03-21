#pragma once

#include "GraphSearchSpecialization.h"

namespace GraphBasedPlanning {
	class GraphSearchCentralizedOptimalPolymorphic : private GraphSearchSpecialization {
		Node<FLOATING_POINT_TYPE> const *sol = nullptr;

	   protected:
		[[nodiscard]] Node<FLOATING_POINT_TYPE> const *find_solution(ColMajorMatrixAccessor<FLOATING_POINT_TYPE> const &x0, std::vector<std::uint8_t> const &trim_indices) final {
			std::priority_queue<Node<FLOATING_POINT_TYPE> *, std::vector<Node<FLOATING_POINT_TYPE> *>, Node<FLOATING_POINT_TYPE>::priority_queue_comparison> pq;

			std::pmr::unsynchronized_pool_resource upr;
			std::pmr::polymorphic_allocator pa{&upr};

			Node<FLOATING_POINT_TYPE> *root = Node<FLOATING_POINT_TYPE>::create_root_node_with_polymorphic_allocator(trim_indices.begin(), &x0(0, 0), &x0(0, 1), &x0(0, 2), pa);

			std::uint8_t *const next_trims = pa.allocate_object<std::uint8_t>(n_vehicles());
			std::uint8_t *const sizes = pa.allocate_object<std::uint8_t>(n_vehicles());
			unsigned int *const multipliers = pa.allocate_object<unsigned int>(n_vehicles());

			/*{  // leider wird die unsynchronized_pool_resource früher vom Stack entfernt als das lambda und das führt zu einem Absturz von matlab --> fix: lambda in einen scope
			    auto expand = [&sizes, &multipliers, &next_trims, &pa, &pq, this](Node<FLOATING_POINT_TYPE> *const node) {
			        std::uint8_t const next_step = node->k() + 1U;

			        // calculate sizes
			        if (next_step < n_hp) {
			            for (unsigned int i = 0; i < n_vehicles; ++i) {
			                sizes[i] = _reachability_list[node->trim(i)].size();
			            }
			        } else {
			            for (unsigned int i = 0; i < n_vehicles; ++i) {
			                sizes[i] = _reachability_list_last[node->trim(i)].size();
			            }
			        }

			        // calculate multipliers
			        multipliers[0] = 1U;
			        for (unsigned int i = 1; i < n_vehicles; ++i) {
			            multipliers[i] = multipliers[i - 1] * sizes[i - 1];
			        }
			        unsigned int const n_children = multipliers[n_vehicles - 1] * sizes[n_vehicles - 1];

			        // create children of parent node
			        node->create_children_with_polymorphic_allocator(n_children, pa);

			        // expand node
			        if (next_step < n_hp) {
			            for (unsigned int k = 0; k < n_children; ++k) {
			                for (unsigned int j = 0; j < n_vehicles; ++j) {
			                    next_trims[j] = _reachability_list[node->trim(j)][k / multipliers[j] % sizes[j]];
			                }

			                Node<FLOATING_POINT_TYPE> *const new_node = Node<FLOATING_POINT_TYPE>::create_node_with_polymorphic_allocator(node->g(), node->h(), node, next_step, pa);
			                init_new_node(node, next_trims, next_step, new_node);

			                node->children(k) = new_node;
			                pq.push(new_node);
			            }
			        } else if (next_step == n_hp) {
			            for (unsigned int k = 0; k < n_children; ++k) {
			                for (unsigned int j = 0; j < n_vehicles; ++j) {
			                    next_trims[j] = _reachability_list_last[node->trim(j)][k / multipliers[j] % sizes[j]];
			                }
			                Node<FLOATING_POINT_TYPE> *const new_node = Node<FLOATING_POINT_TYPE>::create_node_with_polymorphic_allocator(node->g(), node->h(), node, next_step, pa);
			                init_new_node(node, next_trims, next_step, new_node);

			                node->children(k) = new_node;
			                pq.push(new_node);
			            }
			        }
			    };

			    expand(root);

			    while (true) {
			        Node<FLOATING_POINT_TYPE> *const current_node = pq.top();
			        pq.pop();

			        if (!is_path_valid(current_node)) {
			            continue;
			        }

			        if (is_target_reached(current_node)) [[unlikely]] {
			            sol = copy_solution(current_node);
			            break;
			        }

			        expand(current_node);
			    }
			}

			return sol;*/

			expand_node(next_trims, sizes, multipliers, pa, pq, root);

			while (true) {
				Node<FLOATING_POINT_TYPE> *const current_node = pq.top();
				pq.pop();

				if (!is_path_valid(current_node)) {
					continue;
				}

				if (is_target_reached(current_node)) [[unlikely]] {
					return sol = copy_solution(current_node);
				}

				expand_node(next_trims, sizes, multipliers, pa, pq, current_node);
			}
		}

		void clean() final { delete_sol(sol); }

	   private:
		void delete_sol(Node<FLOATING_POINT_TYPE> const *const node) const {
			if (node) {
				delete_sol(node->parent());
				delete node;
			}
		}

		[[nodiscard]] Node<FLOATING_POINT_TYPE> const *copy_solution(Node<FLOATING_POINT_TYPE> const *const node) const {
			if (node->k() > 0) {
				Node<FLOATING_POINT_TYPE> const *parent = copy_solution(node->parent());
				return Node<FLOATING_POINT_TYPE>::create_copy(node, parent);
			} else {
				return Node<FLOATING_POINT_TYPE>::create_copy(node, nullptr);
			}
		}

		void expand_node(std::uint8_t *const next_trims, std::uint8_t *const sizes, unsigned int *const multipliers, std::pmr::polymorphic_allocator<> &pa,
		    std::priority_queue<Node<FLOATING_POINT_TYPE> *, std::vector<Node<FLOATING_POINT_TYPE> *>, Node<FLOATING_POINT_TYPE>::priority_queue_comparison> &pq, Node<FLOATING_POINT_TYPE> *const node) const {
			std::uint8_t const next_step = node->k() + 1U;

			// calculate sizes
			if (next_step < n_hp()) {
				for (unsigned int i = 0; i < n_vehicles(); ++i) {
					sizes[i] = _reachability_list[node->trim(i)].size();
				}
			} else {
				for (unsigned int i = 0; i < n_vehicles(); ++i) {
					sizes[i] = _reachability_list_last[node->trim(i)].size();
				}
			}

			// calculate multipliers
			multipliers[0] = 1U;
			for (unsigned int i = 1; i < n_vehicles(); ++i) {
				multipliers[i] = multipliers[i - 1] * sizes[i - 1];
			}
			unsigned int const n_children = multipliers[n_vehicles() - 1] * sizes[n_vehicles() - 1];

			// create children of parent node
			node->create_children_with_polymorphic_allocator(n_children, pa);

			// expand node
			if (next_step < n_hp()) {
				for (unsigned int k = 0; k < n_children; ++k) {
					for (unsigned int j = 0; j < n_vehicles(); ++j) {
						next_trims[j] = _reachability_list[node->trim(j)][k / multipliers[j] % sizes[j]];
					}

					Node<FLOATING_POINT_TYPE> *const new_node = Node<FLOATING_POINT_TYPE>::create_node_with_polymorphic_allocator(node->g(), node, next_step, pa);
					init_new_node(node, next_trims, next_step, new_node);

					node->children(k) = new_node;
					pq.push(new_node);
				}
			} else if (next_step == n_hp()) {
				for (unsigned int k = 0; k < n_children; ++k) {
					for (unsigned int j = 0; j < n_vehicles(); ++j) {
						next_trims[j] = _reachability_list_last[node->trim(j)][k / multipliers[j] % sizes[j]];
					}
					Node<FLOATING_POINT_TYPE> *const new_node = Node<FLOATING_POINT_TYPE>::create_node_with_polymorphic_allocator(node->g(), node, next_step, pa);
					init_new_node(node, next_trims, next_step, new_node);

					node->children(k) = new_node;
					pq.push(new_node);
				}
			}
		}
	};
}  // namespace GraphBasedPlanning