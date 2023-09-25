#pragma once

#include <array>
#include <concepts>
#include <cstdint>

#include "Node.h"

namespace GraphBasedPlanning {
	template <unsigned int n_vehicles>
	class ChildrenNode : public Node<n_vehicles> {
		unsigned int _children_size;
		ChildrenNode** _children;

	   public:
		ChildrenNode(std::array<std::uint8_t, n_vehicles> const&& trims_, std::array<double, n_vehicles> const&& xs_, std::array<double, n_vehicles> const&& ys_, std::array<double, n_vehicles> const&& yaws_)
		    : Node<n_vehicles>(std::forward<decltype(trims_)>(trims_), std::forward<decltype(xs_)>(xs_), std::forward<decltype(ys_)>(ys_), std::forward<decltype(yaws_)>(yaws_)), _children_size(0), _children(nullptr) {}

		ChildrenNode(ChildrenNode* const parent_, double const g_, std::uint8_t const k_)
		    : Node<n_vehicles>(std::forward<decltype(parent_)>(parent_), std::forward<decltype(g_)>(g_), std::forward<decltype(k_)>(k_)), _children_size(0), _children(nullptr) {}

		explicit ChildrenNode(Node<n_vehicles> root) : Node<n_vehicles>(root), _children_size(0), _children(nullptr) {}

		unsigned int& children_size() { return _children_size; }
		[[nodiscard]] unsigned int children_size() const { return _children_size; }

		ChildrenNode*& child(std::integral auto const index) { return _children[index]; }
		[[nodiscard]] ChildrenNode const* child(std::integral auto const index) const { return _children[index]; }

		[[nodiscard]] ChildrenNode* baby(std::integral auto const index) { return _children[index]; }

		void create_children(std::integral auto const size) {
			_children_size = size;
			_children = new ChildrenNode*[size];
			std::fill(_children, _children + size, nullptr);
		}

		~ChildrenNode() override {
			for (auto i = 0; i < _children_size; ++i) {
				delete _children[i];
			}

			delete[] _children;
		}
	};
}  // namespace GraphBasedPlanning