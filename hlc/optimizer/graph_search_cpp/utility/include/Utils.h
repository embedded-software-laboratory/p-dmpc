#pragma once

#include <array>
#include <concepts>
#include <tuple>
#include <type_traits>

namespace Utils {
	template <std::integral T, std::unsigned_integral T2>
	constexpr T ceiling_integer_division(T x, T2 y) {
		return x / y + (x % y > 0);
	}

	template <std::integral T, std::unsigned_integral T2>
	constexpr T powi(T x, T2 p) {
		T ret = 1;
		for (auto i = 0; i < p; ++i) ret *= x;
		return ret;
	};

	template <std::unsigned_integral T>
	std::make_signed<T>::type interleaved_positive_and_negative_integers(T n) {
		return powi(static_cast<std::make_signed<T>::type>(-1), n + 1) * ceiling_integer_division(static_cast<std::make_signed<T>::type>(n), 2U);
	}

	auto to_tuple = []<typename T, std::size_t N>(std::array<T, N> const arr) {
		auto to_tuple_impl = [&arr]<std::size_t... I>(std::index_sequence<I...>) { return std::make_tuple(arr[I]...); };
		return to_tuple_impl(std::make_index_sequence<N>{});
	};
}  // namespace Utils