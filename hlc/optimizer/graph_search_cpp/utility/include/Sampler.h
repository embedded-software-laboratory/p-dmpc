#pragma once

#include <array>
#include <concepts>
#include <random>
#include <numeric>

// N should be >= n_trims
template <std::integral auto N = 22>
requires(N >= 1) class Sampler {
	static constexpr std::array<std::uniform_int_distribution<>, N> make_distributions_array() {
		auto produce_distributions_array = []<typename T, T... I>(std::integer_sequence<T, I...>) -> std::array<std::uniform_int_distribution<T>, N> { return {std::uniform_int_distribution<T>(static_cast<T>(0), I)...}; };

		return produce_distributions_array(std::make_integer_sequence<std::uniform_int_distribution<>::result_type, N>{});
	}

   protected:
	std::integral auto sample(std::integral auto size) {
		thread_local static std::random_device rd;
		thread_local static std::mt19937 mt{rd()};
		thread_local static std::array distributions{make_distributions_array()};

		return distributions[size - 1](mt);
	}
};

template <std::integral auto N = 22>  // more than 46 would overflow std::uint64_t
requires(N >= 1 && N <= 46) class [[deprecated]] OldSampler {
	constexpr int lcm() requires(N <= 22) {
		int ret = 1;
		for (auto i = 2; i <= N; ++i) {
			ret = std::lcm(ret, i);
		}
		return ret;
	}
	constexpr std::int64_t lcm() requires(N > 22 && N <= 42) {
		std::int64_t ret = 1;
		for (auto i = 2; i <= N; ++i) {
			ret = std::lcm(ret, i);
		}
		return ret;
	}
	constexpr std::uint64_t lcm() requires(N > 42 && N <= 46) {
		std::uint64_t ret = 1;
		for (auto i = 2; i <= N; ++i) {
			ret = std::lcm(ret, i);
		}
		return ret;
	}

   protected:
	std::integral auto sample(std::integral auto size) {
		thread_local static std::random_device rd;
		thread_local static std::mt19937 mt{rd()};
		thread_local std::uniform_int_distribution distribution(static_cast<decltype(lcm())>(1), lcm());

		return distribution(mt) % size;
	}
};