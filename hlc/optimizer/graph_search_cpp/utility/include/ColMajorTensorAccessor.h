#pragma once

#include <array>
#include <cassert>
#include <vector>

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