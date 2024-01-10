#pragma once

#include <vec2.h>

#include <array>
#include <concepts>
#include <cstdint>
#include <vector>

#include "Node.h"

namespace GraphBasedPlanning {
	template <unsigned int n_vehicles>
	class IncrementalNode : public Node<n_vehicles> {
	   public:
		static void n_hp(unsigned int const value) { _n_hp = value; }

	   private:
		static unsigned int _n_hp;

		double _h;
		unsigned int _children_size;
		IncrementalNode** _children;

	   public:
		std::array<std::vector<vec2>, n_vehicles> _trajectory_points;  //[n_vehicles][n_hp];
	   private:
		inline static std::array<std::vector<vec2>, n_vehicles> make_trajectory_points() {
			auto produce_trajectory_points = []<unsigned int... I>(std::integer_sequence<unsigned int, I...>) -> std::array<std::vector<vec2>, n_vehicles> { return {std::vector<vec2>(_n_hp + I * 0)...}; };

			return produce_trajectory_points(std::make_integer_sequence<unsigned int, n_vehicles>{});
		}

	   public:
		IncrementalNode(std::array<std::uint8_t, n_vehicles> const&& trims, std::array<double, n_vehicles> const&& xs, std::array<double, n_vehicles> const&& ys, std::array<double, n_vehicles> const&& yaws)
		    : Node<n_vehicles>(std::forward<decltype(trims)>(trims), std::forward<decltype(xs)>(xs), std::forward<decltype(ys)>(ys), std::forward<decltype(yaws)>(yaws)),
		      _h(0.0),
		      _children_size(0),
		      _children(nullptr),
		      _trajectory_points(make_trajectory_points()) {}

		IncrementalNode(IncrementalNode* const parent_, double const g_, std::uint8_t const k_)
		    : Node<n_vehicles>(std::forward<decltype(parent_)>(parent_), std::forward<decltype(g_)>(g_), std::forward<decltype(k_)>(k_)), _h(0.0), _children_size(0), _children(nullptr), _trajectory_points(make_trajectory_points()) {}

		// new root;
		explicit IncrementalNode(IncrementalNode const* const node)
		    : Node<n_vehicles>(nullptr, node->k(), node->trims(), node->xs(), node->ys(), node->yaws(), 0.0), _h(0.0), _children_size(node->_children_size), _children(node->_children), _trajectory_points(node->_trajectory_points) {}

		explicit IncrementalNode(Node<n_vehicles> root) : Node<n_vehicles>(root), _h(0.0), _children_size(0), _children(nullptr), _trajectory_points(make_trajectory_points()) {}

		double& h() { return _h; }  // cost to go
		[[nodiscard]] double h() const { return _h; }

		unsigned int& children_size() { return _children_size; }
		[[nodiscard]] unsigned int children_size() const { return _children_size; }

		IncrementalNode*& child(std::integral auto const index) { return _children[index]; }
		[[nodiscard]] IncrementalNode const* child(std::integral auto const index) const { return _children[index]; }

		struct priority_queue_comparison {
			bool operator()(IncrementalNode const* const lhs, IncrementalNode const* const rhs) const { return lhs->g() + lhs->h() > rhs->g() + rhs->h(); }
		};

		void create_children(std::integral auto const size) {
			_children_size = size;
			_children = new IncrementalNode*[size];
			std::fill(_children, _children + size, nullptr);
		}

		void delete_from_parent() const {
			for (auto i = 0; i < static_cast<IncrementalNode<n_vehicles> const*>(Node<n_vehicles>::parent())->_children_size; ++i) {
				if (static_cast<IncrementalNode<n_vehicles> const*>(Node<n_vehicles>::parent())->_children[i] == this) {
					static_cast<IncrementalNode<n_vehicles> const*>(Node<n_vehicles>::parent())->_children[i] = nullptr;
				}
			}
		}

		vec2& trajectory_point(std::integral auto const vehicle, std::integral auto const index) { return _trajectory_points[vehicle][index]; }
		[[nodiscard]] vec2 trajectory_point(std::integral auto const vehicle, std::integral auto const index) const { return _trajectory_points[vehicle][index]; }

		~IncrementalNode() override {
			for (auto i = 0; i < _children_size; ++i) {
				delete _children[i];
			}

			delete[] _children;
		}
	};

	template <unsigned int n_vehicles>
	unsigned int IncrementalNode<n_vehicles>::_n_hp = 0;

}  // namespace GraphBasedPlanning