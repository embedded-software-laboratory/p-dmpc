#pragma once

#include <cmath>

struct vec2 {
	double x;
	double y;

	friend inline constexpr vec2 operator-(vec2 const v) noexcept { return vec2{-v.x, -v.y}; }
	friend inline constexpr vec2 operator+(vec2 const v) noexcept { return vec2{+v.x, +v.y}; }
	friend inline constexpr vec2 operator-(vec2 const a, vec2 const b) noexcept { return vec2{a.x - b.x, a.y - b.y}; }
	friend inline constexpr vec2 operator+(vec2 const a, vec2 const b) noexcept { return vec2{a.x + b.x, a.y + b.y}; }
	friend inline constexpr vec2 operator/(vec2 const v, double const divisor) noexcept { return vec2{v.x / divisor, v.y / divisor}; }
	friend inline constexpr vec2 operator*(vec2 const v, double const divisor) noexcept { return vec2{v.x * divisor, v.y * divisor}; }
};

inline constexpr vec2 add(vec2 const a, vec2 const b) noexcept { return a + b; }
inline constexpr vec2 subtract(vec2 const a, vec2 const b) noexcept { return a - b; }
inline constexpr vec2 negate(vec2 const v) noexcept { return -v; }
inline constexpr vec2 divide(vec2 const v, double const divisor) noexcept { return v / divisor; }
inline constexpr vec2 multiply(vec2 const v, double const multiplier) noexcept { return v * multiplier; }
inline constexpr vec2 perpendicular(vec2 const v) noexcept { return vec2{v.y, -v.x}; }
inline constexpr double dotProduct(vec2 const a, vec2 const b) noexcept { return a.x * b.x + a.y * b.y; }
inline constexpr double lengthSquared(vec2 const v) noexcept { return v.x * v.x + v.y * v.y; }
inline double length(vec2 const v) noexcept { return std::sqrt(v.x * v.x + v.y * v.y); }
inline constexpr double distanceSquared(vec2 const v, vec2 const w) noexcept { return (v.x - w.x) * (v.x - w.x) + (v.y - w.y) * (v.y - w.y); }
inline double distance(vec2 const v, vec2 const w) noexcept { return std::sqrt((v.x - w.x) * (v.x - w.x) + (v.y - w.y) * (v.y - w.y)); }
inline vec2 get_point_along_linestrip_with_distance_from_point(vec2 const from, vec2 const to, double const distance) {
	vec2 direction = subtract(to, from);
	double const length_direction = length(direction);
	direction.x /= length_direction;
	direction.y /= length_direction;

	return add(from, multiply(direction, distance));
}