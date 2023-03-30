#pragma once

#include <array>
#include <cassert>
#include <concepts>
#include <cstdint>
#include <functional>
#include <numeric>
#include <span>
#include <vector>

/*
 * This class represent multi-dimensional data. The data is stored as a single std::vector that is expanded to access multiple dimensions.
 */
template <typename T>
class ColMajorAccessor : public std::vector<T> {
	std::vector<uint8_t> _sizes;

   public:
	ColMajorAccessor() : std::vector<T>() {}

	ColMajorAccessor(std::vector<T> &&vec, std::vector<uint8_t> &&sizes) : std::vector<T>(vec), _sizes(sizes) {}

	// data access
	template <typename... IndexType>
	// requires(std::is_convertible<IndexType..., unsigned int>())
	[[nodiscard]] constexpr T &operator()(IndexType const... indices) {
		assert(sizeof...(indices) == _sizes.size());

		unsigned int index = 0;
		for (auto it = _sizes.cbegin(); uint8_t const e : {(unsigned int)(indices)...}) {
			index += std::reduce(_sizes.cbegin(), it++, 1U, std::multiplies<unsigned int>{}) * e;
		}
		return std::vector<T>::operator[](index);
	}

	// data access
	template <typename... IndexType>
	// requires(std::is_convertible<IndexType..., unsigned int>())
	[[nodiscard]] constexpr T const &operator()(IndexType const... indices) const {
		assert(sizeof...(indices) == _sizes.size());

		unsigned int index = 0;
		for (auto it = _sizes.cbegin(); uint8_t const e : {(unsigned int)(indices)...}) {
			index += std::reduce(_sizes.cbegin(), it++, 1U, std::multiplies<unsigned int>{}) * e;
		}
		return std::vector<T>::operator[](index);
	}

	[[nodiscard]] constexpr uint8_t size_of_dimension(unsigned int dimension) const {
		assert(dimension < _sizes.size());

		return _sizes[dimension];
	}
};

/*
 * This class represent 2-dimensional data. The data is stored as a single std::vector that is expanded to access multiple dimensions.
 */
template <typename T>
class ColMajorMatrixAccessor : public std::vector<T> {
	// is not std::size_t, because you should not do calculations with a std::size_t type
	std::array<unsigned int, 2> _sizes{0U, 0U};

   public:
	ColMajorMatrixAccessor() : std::vector<T>() {}
	ColMajorMatrixAccessor(std::vector<unsigned int> const &&sizes) : std::vector<T>(sizes[0] * sizes[1], 0U), _sizes({static_cast<unsigned int>(sizes[0]), static_cast<unsigned int>(sizes[1])}) {}

	template <typename SIZE>
	    requires std::is_convertible_v<SIZE, unsigned int>
	ColMajorMatrixAccessor(std::vector<SIZE> const &&sizes) : std::vector<T>(sizes[0] * sizes[1], 0U), _sizes({static_cast<unsigned int>(sizes[0]), static_cast<unsigned int>(sizes[1])}) {}

	template <typename SIZE>
	    requires std::is_convertible_v<SIZE, unsigned int>
	ColMajorMatrixAccessor(std::vector<T> &&vec, std::vector<SIZE> const sizes) : std::vector<T>(vec), _sizes({static_cast<unsigned int>(sizes[0]), static_cast<unsigned int>(sizes[1])}) {}

	template <typename SIZE>
	    requires std::is_convertible_v<SIZE, unsigned int>
	ColMajorMatrixAccessor(std::vector<T> &&vec, std::initializer_list<SIZE> const &&sizes) : std::vector<T>(vec), _sizes({static_cast<unsigned int>(*sizes.begin()), static_cast<unsigned int>(*(sizes.begin() + 1))}) {}

	// data access
	[[nodiscard]] constexpr T &operator()(unsigned int const index1, unsigned int const index2) { return std::vector<T>::operator[](index1 + _sizes[0] * index2); }

	// data access
	[[nodiscard]] constexpr T const &operator()(unsigned int const index1, unsigned int const index2) const { return std::vector<T>::operator[](index1 + _sizes[0] * index2); }

	[[nodiscard]] constexpr unsigned int size_of_dimension(unsigned int dimension) const {
		assert(dimension < _sizes.size());

		return _sizes[dimension];
	}

	// helper function to get dimension vector in a specific data type
	template <typename SIZE>
	    requires std::is_convertible_v<unsigned int, SIZE>
	[[nodiscard]] std::vector<SIZE> get_dim() {
		return std::vector<SIZE>(_sizes.cbegin(), _sizes.cend());
	}
};

/*
 * This class represent 3-dimensional data. The data is stored as a single std::vector that is expanded to access multiple dimensions.
 */
template <typename T>
class ColMajorTensorAccessor : public std::vector<T> {
	// is not std::size_t, because you should not do calculations with a std::size_t type
	std::array<unsigned int, 3> _sizes{0U, 0U, 0U};

   public:
	ColMajorTensorAccessor() : std::vector<T>() {}

	template <typename SIZE>
	    requires std::is_convertible_v<SIZE, unsigned int>
	ColMajorTensorAccessor(std::vector<T> &&vec, std::vector<SIZE> const &sizes) : std::vector<T>(vec), _sizes({static_cast<unsigned int>(sizes[0]), static_cast<unsigned int>(sizes[1]), static_cast<unsigned int>(sizes[2])}) {}

	// data access
	[[nodiscard]] constexpr T &operator()(unsigned int const index1, unsigned int const index2, unsigned int const index3) { return std::vector<T>::operator[](index1 + _sizes[0] * index2 + _sizes[0] * _sizes[1] * index3); }

	// data access
	[[nodiscard]] constexpr T const &operator()(unsigned int const index1, unsigned int const index2, unsigned int const index3) const { return std::vector<T>::operator[](index1 + _sizes[0] * index2 + _sizes[0] * _sizes[1] * index3); }

	[[nodiscard]] constexpr unsigned int size_of_dimension(unsigned int dimension) const {
		assert(dimension < _sizes.size());

		return _sizes[dimension];
	}
};