#pragma once

#include <boost/heap/priority_queue.hpp>

#include "GraphSearchSpecialization.h"

namespace GraphBasedPlanning {
	class GraphSearchCentralizedParallelNaiveMonteCarlo : private GraphSearchSpecialization {
		Node<FLOATING_POINT_TYPE> const *sol = nullptr;
		static std::uint64_t const n_shots = 1000000U;
		static unsigned int const n_threads = 4U;

		void thread_function(Node<FLOATING_POINT_TYPE> const **const result, Node<FLOATING_POINT_TYPE> *root) {
			boost::heap::priority_queue<Node<FLOATING_POINT_TYPE> const *, boost::heap::compare<Node<FLOATING_POINT_TYPE>::priority_queue_comparison>> pq;
			std::uint8_t *const next_trims = new std::uint8_t[n_vehicles()];

			for (unsigned int i = 0; i < n_shots / n_threads; ++i) {
				pq.push(random_expand(root, next_trims));
			}

			delete[] next_trims;

			Node<FLOATING_POINT_TYPE> const *solution = nullptr;
			while (!pq.empty()) {
				solution = pq.top();
				pq.pop();

				if (check_path(solution)) break;

				delete_node_and_parents(solution);
			}

			for (Node<FLOATING_POINT_TYPE> const *e : pq) {
				delete_node_and_parents(e);
			}

			*result = solution;
		}

	   protected:
		[[nodiscard]] Node<FLOATING_POINT_TYPE> const *find_solution(ColMajorMatrixAccessor<FLOATING_POINT_TYPE> const &x0, std::vector<std::uint8_t> const &trim_indices) final {
			Node<FLOATING_POINT_TYPE> *root = Node<FLOATING_POINT_TYPE>::create_root_node(trim_indices.begin(), &x0(0, 0), &x0(0, 1), &x0(0, 2));

			std::vector<Node<FLOATING_POINT_TYPE> const *> results(n_threads);
			std::vector<std::thread> tasks;

			for (auto &e : results) {
				tasks.emplace_back(&GraphSearchCentralizedParallelNaiveMonteCarlo::thread_function, this, &e, root);
			}

			for (auto &e : tasks) {
				e.join();
			}

			auto it = std::min_element(results.begin(), results.end(), [](Node<FLOATING_POINT_TYPE> const *const lhs, Node<FLOATING_POINT_TYPE> const *const rhs) { return *lhs < *rhs; });
			sol = *it;

			results.erase(it);
			for (Node<FLOATING_POINT_TYPE> const *e : results) {
				delete_node_and_parents(e);
			}

			return sol;
		}

		uint64_t get_n_expanded() { return n_shots; }

		void clean() final {
			for (unsigned int i = 0; i < n_hp(); ++i) {
				auto tmp = sol->parent();
				delete sol;
				sol = tmp;
			}

			delete sol;
		}

	   private:
		[[nodiscard]] Node<FLOATING_POINT_TYPE> *random_expand(Node<FLOATING_POINT_TYPE> *node, std::uint8_t *const next_trims) {
			static auto random = std::mt19937{std::random_device{}()};

			for (unsigned int i = 1; i < n_hp(); ++i) {
				for (unsigned int j = 0; j < n_vehicles(); ++j) {
					std::vector<std::uint8_t> const &sampling_vector = _reachability_list[i-1][node->trim(j)];

					std::sample(sampling_vector.begin(), sampling_vector.end(), next_trims + j, 1, random);
				}

				Node<FLOATING_POINT_TYPE> *const new_node = Node<FLOATING_POINT_TYPE>::create_node(node->g(), node, i);
				init_new_node(node, next_trims, new_node);
				node = new_node;
			}

			for (unsigned int j = 0; j < n_vehicles(); ++j) {
				std::vector<std::uint8_t> const &sampling_vector = _reachability_list[n_hp() - 1][node->trim(j)];

				std::sample(sampling_vector.begin(), sampling_vector.end(), next_trims + j, 1, random);
			}

			Node<FLOATING_POINT_TYPE> *const new_node = Node<FLOATING_POINT_TYPE>::create_node(node->g(), node, n_hp());
			init_new_node(node, next_trims, new_node);
			return new_node;
		}

		void delete_node_and_parents(Node<FLOATING_POINT_TYPE> const *node) const {  // without root
			for (unsigned int i = 0; i < n_hp(); ++i) {
				auto tmp = node->parent();
				delete node;
				node = tmp;
			}
		};

		bool check_path(Node<FLOATING_POINT_TYPE> const *solution) {
			do {
				if (!is_path_valid_centralized(solution)) return false;
				solution = solution->parent();
			} while (solution->k());

			return true;
		}
	};
}  // namespace GraphBasedPlanning