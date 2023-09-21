#pragma once

#include <array>
#include <boost/heap/binomial_heap.hpp>
#include <concepts>
#include <cstdint>
#include <memory_resource>

#include "Node.h"

namespace GraphBasedPlanning {
	class PriorityBasedNode : public Node<1> {
		double _h;
		unsigned int _children_size;
		PriorityBasedNode** _children;
		std::vector<double> _shapes;

	   public:
		PriorityBasedNode(std::array<std::uint8_t, 1> const&& trims_, std::array<double, 1> const&& xs_, std::array<double, 1> const&& ys_, std::array<double, 1> const&& yaws_)
		    : Node<1>(std::forward<decltype(trims_)>(trims_), std::forward<decltype(xs_)>(xs_), std::forward<decltype(ys_)>(ys_), std::forward<decltype(yaws_)>(yaws_)), _h(0.0), _children_size(0), _children(nullptr) {}

		// copy
		PriorityBasedNode(PriorityBasedNode* const parent_, unsigned int const k_, std::array<std::uint8_t, 1> const trims_, std::array<double, 1> const xs_, std::array<double, 1> const ys_,
		    std::array<double, 1> const yaws_, double const g_, double const h_, std::vector<double> const&& shapes_)
		    : Node<1>(std::forward<decltype(parent_)>(parent_), std::forward<decltype(k_)>(k_), std::forward<decltype(trims_)>(trims_), std::forward<decltype(xs_)>(xs_), std::forward<decltype(ys_)>(ys_),
		          std::forward<decltype(yaws_)>(yaws_), std::forward<decltype(g_)>(g_)),
		      _h(h_),
		      _shapes(std::move(shapes_)) {}

		PriorityBasedNode(PriorityBasedNode* const parent_, double const g_, std::uint8_t const k_)
		    : Node<1>(std::forward<decltype(parent_)>(parent_), std::forward<decltype(g_)>(g_), std::forward<decltype(k_)>(k_)), _h(0.0), _children_size(0), _children(nullptr) {}

		explicit PriorityBasedNode(Node<1> root) : Node<1>(root), _h(0.0), _children_size(0), _children(nullptr) {}

		double& h() { return _h; }  // cost to go
		[[nodiscard]] double h() const { return _h; }

		unsigned int& children_size() { return _children_size; }
		[[nodiscard]] unsigned int children_size() const { return _children_size; }

		PriorityBasedNode *& child(std::integral auto const index) { return _children[index]; }
		[[nodiscard]] PriorityBasedNode const* child(std::integral auto const index) const { return _children[index]; }

		std::vector<double>& shapes() { return _shapes; }
		[[nodiscard]] std::vector<double> shapes() const { return _shapes; }

		struct priority_queue_comparison {
			bool operator()(PriorityBasedNode const* const lhs, PriorityBasedNode const* const rhs) const { return lhs->g() + lhs->h() > rhs->g() + rhs->h(); }
		};

		void create_children(std::integral auto const size) {
			_children_size = size;
			_children = new PriorityBasedNode *[size];
			std::fill(_children, _children + size, nullptr);
		}

		~PriorityBasedNode() override {
			for (auto i = 0; i < _children_size; ++i) {
				delete _children[i];
			}

			if (heap_handle().has_value()) {
				heap()->erase(heap_handle().value());
			}

			delete[] _children;
		}

		typedef typename boost::heap::binomial_heap<PriorityBasedNode*, boost::heap::mutable_<true>, boost::heap::compare<PriorityBasedNode::priority_queue_comparison>> heap_t;
		inline static heap_t* _heap = nullptr;

		typedef typename heap_t::handle_type heap_handle_t;
		std::optional<heap_handle_t> _heap_handle;

		[[nodiscard]] std::optional<heap_handle_t>& heap_handle() { return _heap_handle; }
		void set_heap_handle(heap_handle_t heap_handle) { _heap_handle = heap_handle; }
		void reset_heap_handle() { _heap_handle.reset(); }
		[[nodiscard]] static heap_t*& heap() { return _heap; }

		inline static unsigned int _k_offset = 0;

		[[nodiscard]] unsigned int k() const final { return Node<1>::k() - _k_offset; }
		static unsigned int &k_offset() { return _k_offset; }
		static void increase_k_offset() { _k_offset++; }
		static void reset_k_offset() { _k_offset = 0; }
	};
}  // namespace GraphBasedPlanning