#pragma once

#include "GraphSearchSpecialization.h"

namespace GraphBasedPlanning {
	class GraphSearchCentralizedOptimalMemorySaving : private GraphSearchSpecialization {
		Node<FLOATING_POINT_TYPE> *root = nullptr;
		uint64_t n_expanded;

	   protected:
		[[nodiscard]] Node<FLOATING_POINT_TYPE> const *find_solution(ColMajorMatrixAccessor<FLOATING_POINT_TYPE> const &x0, std::vector<std::uint8_t> const &trim_indices) final {
			std::priority_queue<Node<FLOATING_POINT_TYPE> *, std::vector<Node<FLOATING_POINT_TYPE> *>, Node<FLOATING_POINT_TYPE>::priority_queue_comparison> pq;

			root = Node<FLOATING_POINT_TYPE>::create_root_node(trim_indices.begin(), &x0(0, 0), &x0(0, 1), &x0(0, 2));
			n_expanded = 1;

			expand_node(root, [&pq](Node<FLOATING_POINT_TYPE> *node) -> void { pq.push(node); });

			while (true) {
				if (!pq.empty()) {
					Node<FLOATING_POINT_TYPE> *const current_node = pq.top();
					pq.pop();

					if (!is_path_valid_centralized(current_node)) {
						current_node->delete_and_delete_from_parent();

						continue;
					}

					if (is_target_reached(current_node)) [[unlikely]] {
						return current_node;
					}

					// if (*current_node == *old_node) {
					//	throw MexException("is exhausted!");
					// }

					expand_node(current_node, [&pq](Node<FLOATING_POINT_TYPE> *node) -> void { pq.push(node); });
				} else {
					return nullptr;
				}
			}
		}

		void clean() final { delete root; }

		uint64_t get_n_expanded() { return n_expanded; }

	   private:
		void expand_node(Node<FLOATING_POINT_TYPE> *node, const std::function<void(Node<FLOATING_POINT_TYPE> *)> &push) {
			std::uint8_t const next_step = node->k() + 1U;

			std::uint8_t *sizes = get_reachable_states_sizes(&node->trim(0), next_step);
			unsigned int *multipliers = get_reachable_states_multipliers(sizes);

			node->create_children(multipliers[n_vehicles()]);

			if (next_step <= n_hp()) {
				for (unsigned int k = 0; uint8_t const *const next_trims : reachable_states(&node->trim(0), sizes, multipliers, next_step)) {
					Node<FLOATING_POINT_TYPE> *const new_node = Node<FLOATING_POINT_TYPE>::create_node(node->g(), node, next_step);
					init_new_node(node, next_trims, new_node);

					n_expanded++;
					node->children(k++) = new_node;
					push(new_node);
				}
			}
		}

		void expand_node_2(Node<FLOATING_POINT_TYPE> *node, const std::function<void(Node<FLOATING_POINT_TYPE> *)> &push) {
			std::uint8_t const next_step = node->k() + 1U;

			std::uint8_t *sizes = get_reachable_states_sizes(&node->trim(0), next_step);
			unsigned int *multipliers = get_reachable_states_multipliers(sizes);

			node->create_children(multipliers[n_vehicles()]);

			std::uint8_t *const next_trims = new std::uint8_t[n_vehicles()];

			if (next_step <= n_hp()) {
				for (unsigned int k = 0; k < multipliers[n_vehicles()]; ++k) {
					reachable_states(&node->trim(0), sizes, multipliers, next_step, next_trims, k);

					Node<FLOATING_POINT_TYPE> *const new_node = Node<FLOATING_POINT_TYPE>::create_node(node->g(), node, next_step);
					init_new_node(node, next_trims, new_node);

					n_expanded++;
					node->children(k) = new_node;
					push(new_node);
				}
			}

			delete[] next_trims;
			delete[] multipliers;
			delete[] sizes;
		}
	};
}  // namespace GraphBasedPlanning