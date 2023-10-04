#pragma once

#include <array>
#include <cstdint>
#include <concepts>

namespace GraphBasedPlanning {
	template <unsigned int n_vehicles>
	class Node {
	   protected:
		Node* const _parent;  // parent
	   private:
		unsigned int const _k;                        // step
		std::array<std::uint8_t, n_vehicles> _trims;  //[n_vehicles];
		std::array<double, n_vehicles> _xs;           //[n_vehicles];
		std::array<double, n_vehicles> _ys;           //[n_vehicles];
		std::array<double, n_vehicles> _yaws;         //[n_vehicles];
		double _g;                                    // cost to come

	   public:
		[[nodiscard]] Node const* parent() const { return _parent; }
		[[nodiscard]] Node* parent() { return _parent; }

		[[nodiscard]] virtual unsigned int k() const { return _k; }

		std::uint8_t& trim(std::integral auto const index) { return _trims[index]; }
		[[nodiscard]] std::uint8_t trim(std::integral auto const index) const { return _trims[index]; }

		std::array<std::uint8_t, n_vehicles>& trims() { return _trims; }
		[[nodiscard]] std::array<std::uint8_t, n_vehicles> const& trims() const { return _trims; }

		std::array<double, n_vehicles>& xs() { return _xs; }
		[[nodiscard]] std::array<double, n_vehicles> const& xs() const { return _xs; }

		std::array<double, n_vehicles>& ys() { return _ys; }
		[[nodiscard]] std::array<double, n_vehicles> const& ys() const { return _ys; }

		std::array<double, n_vehicles>& yaws() { return _yaws; }
		[[nodiscard]] std::array<double, n_vehicles> const& yaws() const { return _yaws; }

		double& x(std::integral auto const index) { return _xs[index]; }
		[[nodiscard]] double x(std::integral auto const index) const { return _xs[index]; }

		double& y(std::integral auto const index) { return _ys[index]; }
		[[nodiscard]] double y(std::integral auto const index) const { return _ys[index]; }

		double& yaw(std::integral auto const index) { return _yaws[index]; }
		[[nodiscard]] double yaw(std::integral auto const index) const { return _yaws[index]; }

		double& g() { return _g; }
		[[nodiscard]] double g() const { return _g; }

		Node(std::array<std::uint8_t, n_vehicles> const&& trims_, std::array<double, n_vehicles> const&& xs_, std::array<double, n_vehicles> const&& ys_, std::array<double, n_vehicles> const&& yaws_)
		    : _g(0.0), _k(0U), _parent(nullptr), _trims(trims_), _xs(xs_), _ys(ys_), _yaws(yaws_) {}

		Node(Node* const parent_, double const g_, std::uint8_t const k_) : _g(g_), _k(k_), _parent(parent_) {}

		// copy
		Node(Node* const parent_, unsigned int const k_, std::array<std::uint8_t, n_vehicles> const trims_, std::array<double, n_vehicles> const xs_, std::array<double, n_vehicles> const ys_, std::array<double, n_vehicles> const yaws_,
		    double const g_)
		    : _parent(parent_), _k(k_), _trims(trims_), _xs(xs_), _ys(ys_), _yaws(yaws_), _g(g_) {}

		virtual ~Node() = default;
	};
}