#pragma once

#include <array>
#include <cassert>
#include <vector>

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
	requires std::is_convertible_v<SIZE, unsigned int> ColMajorMatrixAccessor(std::vector<SIZE> const &&sizes) : std::vector<T>(sizes[0] * sizes[1], 0U), _sizes({static_cast<unsigned int>(sizes[0]), static_cast<unsigned int>(sizes[1])}) {}

	template <typename SIZE>
	requires std::is_convertible_v<SIZE, unsigned int> ColMajorMatrixAccessor(std::vector<T> &&vec, std::vector<SIZE> const sizes) : std::vector<T>(vec), _sizes({static_cast<unsigned int>(sizes[0]), static_cast<unsigned int>(sizes[1])}) {}

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
	requires std::is_convertible_v<unsigned int, SIZE> [[nodiscard]] std::vector<SIZE> get_dim() {
		return std::vector<SIZE>(_sizes.cbegin(), _sizes.cend());
	}
};