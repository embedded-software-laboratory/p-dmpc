#pragma once

#include <array>
#include <concepts>
#include <cstdint>
#include <memory_resource>

#include "Node.h"

namespace GraphBasedPlanning {
	template <unsigned int n_vehicles>
	class OptimalNode : public Node<n_vehicles> {
		double _h;
		unsigned int _children_size;
		OptimalNode const** _children;

	   public:
		OptimalNode(std::array<std::uint8_t, n_vehicles> const&& trims_, std::array<double, n_vehicles> const&& xs_, std::array<double, n_vehicles> const&& ys_, std::array<double, n_vehicles> const&& yaws_)
		    : Node<n_vehicles>(std::forward<decltype(trims_)>(trims_), std::forward<decltype(xs_)>(xs_), std::forward<decltype(ys_)>(ys_), std::forward<decltype(yaws_)>(yaws_)), _h(0.0), _children_size(0), _children(nullptr) {}

		// copy
		OptimalNode(OptimalNode* const parent_, unsigned int const k_, std::array<std::uint8_t, n_vehicles> const trims_, std::array<double, n_vehicles> const xs_, std::array<double, n_vehicles> const ys_,
		    std::array<double, n_vehicles> const yaws_, double const g_, double const h_)
		    : Node<n_vehicles>(std::forward<decltype(parent_)>(parent_), std::forward<decltype(k_)>(k_), std::forward<decltype(trims_)>(trims_), std::forward<decltype(xs_)>(xs_), std::forward<decltype(ys_)>(ys_),
		          std::forward<decltype(yaws_)>(yaws_), std::forward<decltype(g_)>(g_)),
		      _h(h_) {}

		OptimalNode(OptimalNode* const parent_, double const g_, std::uint8_t const k_)
		    : Node<n_vehicles>(std::forward<decltype(parent_)>(parent_), std::forward<decltype(g_)>(g_), std::forward<decltype(k_)>(k_)), _h(0.0), _children_size(0), _children(nullptr) {}

		explicit OptimalNode(Node<n_vehicles> root) : Node<n_vehicles>(root), _h(0.0), _children_size(0), _children(nullptr) {}

		double& h() { return _h; }  // cost to go
		[[nodiscard]] double h() const { return _h; }

		unsigned int& children_size() { return _children_size; }
		[[nodiscard]] unsigned int children_size() const { return _children_size; }

		OptimalNode const*& child(std::integral auto const index) { return _children[index]; }
		[[nodiscard]] OptimalNode const* child(std::integral auto const index) const { return _children[index]; }

		struct priority_queue_comparison {
			bool operator()(OptimalNode const* const lhs, OptimalNode const* const rhs) const { return lhs->g() + lhs->h() > rhs->g() + rhs->h(); }
		};

		void create_children(std::integral auto const size) {
			_children_size = size;
			_children = new OptimalNode const*[size];
			std::fill(_children, _children + size, nullptr);
		}
		void create_children(std::integral auto const size, std::pmr::polymorphic_allocator<>& pa) {
			_children_size = size;
			_children = pa.allocate_object<OptimalNode const*>(size);
			std::fill(_children, _children + size, nullptr);
		}

		void delete_from_parent() const {
			for (auto i = 0; i < static_cast<OptimalNode<n_vehicles> const*>(Node<n_vehicles>::parent())->_children_size; ++i) {
				if (static_cast<OptimalNode<n_vehicles> const*>(Node<n_vehicles>::parent())->_children[i] == this) {
					static_cast<OptimalNode<n_vehicles> const*>(Node<n_vehicles>::parent())->_children[i] = nullptr;
				}
			}
		}

		~OptimalNode() override {
			for (auto i = 0; i < _children_size; ++i) {
				delete _children[i];
			}

			delete[] _children;
		}
	};
}  // namespace GraphBasedPlanning