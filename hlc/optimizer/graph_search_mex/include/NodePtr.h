#pragma once

#include <cstddef>
#include <cstdint>

namespace GraphBasedPlanning::NodePtr {
	[[deprecated]] static std::ptrdiff_t _k;
	[[deprecated]] static std::ptrdiff_t _g;
	[[deprecated]] static std::ptrdiff_t _h;
	[[deprecated]] static std::ptrdiff_t _parent;
	[[deprecated]] static std::ptrdiff_t _trim;
	[[deprecated]] static std::ptrdiff_t _x;
	[[deprecated]] static std::ptrdiff_t _y;
	[[deprecated]] static std::ptrdiff_t _yaw;
	[[deprecated]] static std::ptrdiff_t _max;
	[[deprecated]] static std::ptrdiff_t _children_size;
	[[deprecated]] static std::ptrdiff_t _children;

	template <typename T>
	[[deprecated]] static void init_static_variables(unsigned int n_vehicles) {
		_k = 0;
		_g = _k + sizeof(T);  // offset, because faster
		_h = _g + sizeof(T);
		_parent = _h + sizeof(T);
		_trim = _parent + sizeof(std::uint8_t *);
		auto tmp = n_vehicles * sizeof(std::uint8_t);
		_x = _trim + tmp + sizeof(T) - (tmp % sizeof(T));
		_y = _x + n_vehicles * sizeof(T);
		_yaw = _y + n_vehicles * sizeof(T);
		_children_size = _yaw + n_vehicles * sizeof(T);
		_children = _children_size + sizeof(unsigned int);
		_max = _children + sizeof(std::uint8_t **);
	}

	[[deprecated]] [[nodiscard]] static std::uint8_t &k(std::uint8_t *const memory) { return *(std::uint8_t *)(memory + _k); }
	[[deprecated]] [[nodiscard]] static std::uint8_t const &k(std::uint8_t const *const memory) { return *(std::uint8_t *)(memory + _k); }

	template <typename T>
	[[deprecated]] [[nodiscard]] static T &g(std::uint8_t *const memory) {
		return *(T *)(memory + _g);
	}
	template <typename T>
	[[deprecated]] [[nodiscard]] static T const &g(std::uint8_t const *const memory) {
		return *(T *)(memory + _g);
	}

	template <typename T>
	[[deprecated]] [[nodiscard]] static T &h(std::uint8_t *const memory) {
		return *(T *)(memory + _h);
	}
	template <typename T>
	[[deprecated]] [[nodiscard]] static T const &h(std::uint8_t const *const memory) {
		return *(T *)(memory + _h);
	}

	[[deprecated]] [[nodiscard]] static std::uint8_t *&parent(std::uint8_t *const memory) { return *(std::uint8_t **)(memory + _parent); }
	[[deprecated]] [[nodiscard]] static std::uint8_t *const &parent(std::uint8_t const *const memory) { return *(std::uint8_t **)(memory + _parent); }

	[[deprecated]] [[nodiscard]] static std::uint8_t &trim(std::uint8_t *const memory, std::size_t const vehicle) { return *(std::uint8_t *)(memory + _trim + sizeof(std::uint8_t) * vehicle); }
	[[deprecated]] [[nodiscard]] static std::uint8_t const &trim(std::uint8_t const *const memory, std::size_t const vehicle) { return *(std::uint8_t *)(memory + _trim + sizeof(std::uint8_t) * vehicle); }
	[[deprecated]] [[nodiscard]] static std::uint8_t *trim_ptr(std::uint8_t *const memory) { return (std::uint8_t *)(memory + _trim); }
	[[deprecated]] [[nodiscard]] static std::uint8_t const *trim_ptr(std::uint8_t const *const memory) { return (std::uint8_t *)(memory + _trim); }

	template <typename T>
	[[deprecated]] [[nodiscard]] static T &x(std::uint8_t *const memory, std::size_t const vehicle) {
		return *(T *)(memory + _x + sizeof(T) * vehicle);
	}
	template <typename T>
	[[deprecated]] [[nodiscard]] static T const &x(std::uint8_t const *const memory, std::size_t const vehicle) {
		return *(T *)(memory + _x + sizeof(T) * vehicle);
	}
	template <typename T>
	[[deprecated]] [[nodiscard]] static T *x_ptr(std::uint8_t *const memory) {
		return (T *)(memory + _x);
	}
	template <typename T>
	[[deprecated]] [[nodiscard]] static T const *x_ptr(std::uint8_t const *const memory) {
		return (T *)(memory + _x);
	}

	template <typename T>
	[[deprecated]] [[nodiscard]] static T &y(std::uint8_t *const memory, std::size_t const vehicle) {
		return *(T *)(memory + _y + sizeof(T) * vehicle);
	}
	template <typename T>
	[[deprecated]] [[nodiscard]] static T const &y(std::uint8_t const *const memory, std::size_t const vehicle) {
		return *(T *)(memory + _y + sizeof(T) * vehicle);
	}
	template <typename T>
	[[deprecated]] [[nodiscard]] static T *y_ptr(std::uint8_t *memory) {
		return (T *)(memory + _y);
	}
	template <typename T>
	[[deprecated]] [[nodiscard]] static T const *y_ptr(std::uint8_t const *const memory) {
		return (T *)(memory + _y);
	}

	template <typename T>
	[[deprecated]] [[nodiscard]] static T &yaw(std::uint8_t *const memory, std::size_t const vehicle) {
		return *(T *)(memory + _yaw + sizeof(T) * vehicle);
	}
	template <typename T>
	[[deprecated]] [[nodiscard]] static T const &yaw(std::uint8_t const *const memory, std::size_t const vehicle) {
		return *(T *)(memory + _yaw + sizeof(T) * vehicle);
	}
	template <typename T>
	[[deprecated]] [[nodiscard]] static T *yaw_ptr(std::uint8_t *const memory) {
		return (T *)(memory + _yaw);
	}
	template <typename T>
	[[deprecated]] [[nodiscard]] static T const *yaw_ptr(std::uint8_t const *const memory) {
		return (T *)(memory + _yaw);
	}

	[[deprecated]] [[nodiscard]] static unsigned int &children_size(std::uint8_t *const memory) { return *(unsigned int *)(memory + _children_size); }
	[[deprecated]] [[nodiscard]] static unsigned int const &children_size(std::uint8_t const *const memory) { return *(unsigned int *)(memory + _children_size); }

	[[deprecated]] [[nodiscard]] static std::uint8_t **&children_ptr(std::uint8_t *const memory) { return *(std::uint8_t ***)(memory + _children); }
	[[deprecated]] [[nodiscard]] static std::uint8_t **const &children_ptr(std::uint8_t const *const memory) { return *(std::uint8_t ***)(memory + _children); }
	[[deprecated]] [[nodiscard]] static std::uint8_t *&children(std::uint8_t *const memory, std::size_t const child) { return children_ptr(memory)[child]; }
	[[deprecated]] [[nodiscard]] static std::uint8_t *const &children(std::uint8_t const *const memory, std::size_t const child) { return children_ptr(memory)[child]; }

	[[deprecated]] static void create_children(std::uint8_t *const memory, std::size_t const size) {
		auto ptr = new std::uint8_t *[size] { nullptr };
		children_ptr(memory) = ptr;
		children_size(memory) = size;
	}

	template <typename T>
	[[deprecated]] [[nodiscard]] std::uint8_t *create_root_node(std::size_t const n_vehicles) {
		init_static_variables<T>(n_vehicles);
		auto *memory = new std::uint8_t[_max];
		k(memory) = 0;
		g<T>(memory) = 0;
		h<T>(memory) = 0;
		children_size(memory) = 0;
		return memory;
	}

	template <typename T>
	[[deprecated]] [[nodiscard]] std::uint8_t *create_node(std::uint8_t *const parent_, std::uint8_t const k_, T const g_, T const h_) {
		auto *memory = new std::uint8_t[_max];
		parent(memory) = parent_;
		k(memory) = k_;
		g<T>(memory) = g_;
		h<T>(memory) = h_;
		children_size(memory) = 0;
		return memory;
	}

	[[deprecated]] inline void delete_node(const std::uint8_t *const node) {
		for (unsigned int i = 0; i < children_size(node); ++i) {
			/*if (children(node, i))*/ delete_node(children(node, i));
		}

		if (children_size(node)) delete[] children_ptr(node);
		delete[] node;
	}
}  // namespace GraphBasedPlanning::NodePtr