#pragma once

#include <array>
#include <boost/heap/priority_queue.hpp>

#include "GraphSearchSpecialization.h"

namespace GraphBasedPlanning {
	class GraphSearchCentralizedParallelNaiveMonteCarloPolymorphic : private GraphSearchSpecialization {
		static std::uint64_t const n_shots = 1000000U;
		static unsigned int const n_threads = 4U;
		static std::byte *buffers[n_threads];

		void thread_function(Node<FLOATING_POINT_TYPE> const **const result, ColMajorMatrixAccessor<FLOATING_POINT_TYPE> const &x0, std::vector<std::uint8_t> const &trim_indices, unsigned int const id) {
			boost::heap::priority_queue<Node<FLOATING_POINT_TYPE> const *, boost::heap::compare<Node<FLOATING_POINT_TYPE>::priority_queue_comparison>> pq;
			pq.reserve(n_shots / n_threads);

			// set up polymorphic_allocator with monotonic_buffer with exactly required size
			std::size_t const size = (n_shots / n_threads) * Node<FLOATING_POINT_TYPE>::max() * n_hp() + Node<FLOATING_POINT_TYPE>::max() + 1024;
			buffers[id] = new std::byte[size];
			std::pmr::monotonic_buffer_resource mbr(buffers[id], size);
			std::pmr::polymorphic_allocator pa{&mbr};

			Node<FLOATING_POINT_TYPE> *const root = Node<FLOATING_POINT_TYPE>::create_root_node_with_polymorphic_allocator(trim_indices.begin(), &x0(0, 0), &x0(0, 1), &x0(0, 2), pa);

			std::uint8_t *const next_trims = pa.new_object<std::uint8_t>(n_vehicles());
			for (unsigned int i = 0; i < n_shots / n_threads; ++i) {
				pq.push(random_expand(root, next_trims, pa));
			}

			Node<FLOATING_POINT_TYPE> const *solution = nullptr;
			while (!pq.empty()) {
				solution = pq.top();
				pq.pop();

				if (check_path(solution)) break;
			}

			*result = solution;
		}

	   protected:
		[[nodiscard]] Node<FLOATING_POINT_TYPE> const *find_solution(ColMajorMatrixAccessor<FLOATING_POINT_TYPE> const &x0, std::vector<std::uint8_t> const &trim_indices) final {
			Node<FLOATING_POINT_TYPE> const *results[n_threads];
			std::vector<std::thread> tasks;

			unsigned int id;
			for (id = 0; id < n_threads - 1; ++id) {
				tasks.emplace_back(&GraphSearchCentralizedParallelNaiveMonteCarloPolymorphic::thread_function, this, &results[id], x0, trim_indices, id);
			}

			thread_function(&results[id], x0, trim_indices, id);

			for (auto &e : tasks) {
				e.join();
			}

			return *std::min_element(results, results + n_threads, [](Node<FLOATING_POINT_TYPE> const *const lhs, Node<FLOATING_POINT_TYPE> const *const rhs) { return *lhs < *rhs; });
		}

		uint64_t get_n_expanded() {return n_shots; }

		void clean() final {
			for (auto &buffer : buffers) {
				delete[] buffer;
			}
		}

	   private:
		[[nodiscard]] Node<FLOATING_POINT_TYPE> *random_expand(Node<FLOATING_POINT_TYPE> *node, std::uint8_t *const next_trims, std::pmr::polymorphic_allocator<> &pa) const {
			static auto random = std::mt19937{std::random_device{}()};

			for (unsigned int i = 1; i < n_hp(); ++i) {
				for (unsigned int j = 0; j < n_vehicles(); ++j) {
					std::vector<std::uint8_t> const &sampling_vector = _reachability_list[i -1][node->trim(j)];

					std::sample(sampling_vector.begin(), sampling_vector.end(), next_trims + j, 1, random);
				}

				Node<FLOATING_POINT_TYPE> *const new_node = Node<FLOATING_POINT_TYPE>::create_node_with_polymorphic_allocator(node->g(), node, i, pa);
				init_new_node(node, next_trims, new_node);
				node = new_node;
			}

			for (unsigned int j = 0; j < n_vehicles(); ++j) {
				std::vector<std::uint8_t> const &sampling_vector = _reachability_list[n_hp() - 1][node->trim(j)];

				std::sample(sampling_vector.begin(), sampling_vector.end(), next_trims + j, 1, random);
			}

			Node<FLOATING_POINT_TYPE> *const new_node = Node<FLOATING_POINT_TYPE>::create_node_with_polymorphic_allocator(node->g(), node, n_hp(), pa);
			init_new_node(node, next_trims,  new_node);
			return new_node;
		}

		bool check_path(Node<FLOATING_POINT_TYPE> const *solution) const {
			do {
				if (!is_path_valid_centralized(solution)) return false;
				solution = solution->parent();
			} while (solution->k());

			return true;
		}
	};

	std::byte *GraphSearchCentralizedParallelNaiveMonteCarloPolymorphic::buffers[n_threads];
}  // namespace GraphBasedPlanning