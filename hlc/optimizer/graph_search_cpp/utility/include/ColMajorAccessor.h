#pragma once

#include <cassert>
#include <cstdint>
#include <numeric>
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