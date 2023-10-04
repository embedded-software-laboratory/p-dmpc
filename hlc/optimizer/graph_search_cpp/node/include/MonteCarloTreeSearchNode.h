#pragma once

#include <array>
#include <cmath>
#include <concepts>
#include <cstdint>
#include <limits>

#include "Node.h"

namespace GraphBasedPlanning {
	struct UCB1 {
		double w = 0.0;
		double n = 0.0;
	};

	template <unsigned int n_vehicles>
	class MonteCarloTreeSearchNode : public Node<n_vehicles> {
		unsigned int _children_size;
		unsigned int _index;
		MonteCarloTreeSearchNode** _children;
		UCB1* _values;

	   public:
		MonteCarloTreeSearchNode(std::array<std::uint8_t, n_vehicles> const&& trims, std::array<double, n_vehicles> const&& xs, std::array<double, n_vehicles> const&& ys, std::array<double, n_vehicles> const&& yaws)
		    : Node<n_vehicles>(std::forward<decltype(trims)>(trims), std::forward<decltype(xs)>(xs), std::forward<decltype(ys)>(ys), std::forward<decltype(yaws)>(yaws)),
		      _children_size(0),
		      _index((std::numeric_limits<unsigned int>::max)()),
		      _children(nullptr),
		      _values(nullptr) {}

		MonteCarloTreeSearchNode(MonteCarloTreeSearchNode* const parent_, double const g_, std::uint8_t const k_, unsigned int const index)
		    : Node<n_vehicles>(std::forward<decltype(parent_)>(parent_), std::forward<decltype(g_)>(g_), std::forward<decltype(k_)>(k_)), _children_size(0), _index(index), _children(nullptr), _values(nullptr) {}

		explicit MonteCarloTreeSearchNode(Node<n_vehicles> root) : Node<n_vehicles>(root), _children_size(0), _index((std::numeric_limits<unsigned int>::max)()), _children(nullptr), _values(nullptr) {}

		unsigned int& children_size() { return _children_size; }
		[[nodiscard]] unsigned int children_size() const { return _children_size; }

		unsigned int& index() { return _index; }
		[[nodiscard]] unsigned int index() const { return _index; }

		MonteCarloTreeSearchNode*& child(std::integral auto const index) { return _children[index]; }
		[[nodiscard]] MonteCarloTreeSearchNode* child(std::integral auto const index) const { return _children[index]; }

		void create_children(std::integral auto const size) {
			_children_size = size;
			_children = new MonteCarloTreeSearchNode*[size];
			std::fill(_children, _children + size, nullptr);
			_values = new UCB1[size];
		}

		std::signed_integral auto ucb1_index() {
			auto max_index = -1;
			double max = -std::numeric_limits<double>::infinity();
			static double const c = std::sqrt(2);

			for (auto i = 0; i < _children_size; ++i) {
				double const N = static_cast<MonteCarloTreeSearchNode<n_vehicles>* const>(Node<n_vehicles>::_parent)->n(_index);
				double const ucb1 = _values[i].w / (_values[i].n + 1.0) + c * std::sqrt(std::log(N + 1.0) / (_values[i].n + 1.0));
				if (ucb1 > max) {
					max = ucb1;
					max_index = i;
				}
			}

			return max_index;
		}

		void deactivate_child(std::integral auto const index) { _values[index].n = std::numeric_limits<double>::infinity(); }

		double& w(std::integral auto const index) { return _values[index].w; }
		[[nodiscard]] double w(std::integral auto const index) const { return _values[index].w; }

		double& n(std::integral auto const index) { return _values[index].n; }
		[[nodiscard]] double n(std::integral auto const index) const { return _values[index].n; }

		struct priority_queue_comparison {
			bool operator()(MonteCarloTreeSearchNode const* const lhs, MonteCarloTreeSearchNode const* const rhs) const { return lhs->g() > rhs->g(); }
		};

		~MonteCarloTreeSearchNode() {
			for (auto i = 0; i < _children_size; ++i) {
				delete _children[i];
			}

			delete[] _children;
			delete[] _values;
		}
	};
}  // namespace GraphBasedPlanning