#include "../include/CollisionDetection.h"

#include "../include/earcut.h"

namespace mapbox::util {
	template <>
	struct nth<0, vec2> {
		inline static auto get(vec2 const& t) { return t.x; };
	};
	template <>
	struct nth<1, vec2> {
		inline static auto get(vec2 const& t) { return t.y; };
	};
}  // namespace mapbox::util

inline bool CollisionDetection::check_collision_concave(const std::vector<std::array<vec2, 3>>& triangles1, const std::vector<std::array<vec2, 3>>& triangles2) const {
	for (const auto& k : triangles1) {
		for (const auto& l : triangles2) {
			if (check_collision(k.data(), k.size(), l.data(), l.size())) return true;
		}
	}

	return false;
}
std::vector<std::array<vec2, 3>> CollisionDetection::generate_triangles(const std::vector<vec2>& polygon) {
	std::vector<std::array<vec2, 3>> res;

	std::vector<std::vector<vec2>> poly = {polygon};
	std::vector<std::uint32_t> indices = mapbox::earcut<std::uint32_t>(poly);

	for (unsigned int j = 0; j < indices.size(); j += 3) {
		res.push_back({vec2{polygon[indices[j]].x, polygon[indices[j]].y}, vec2{polygon[indices[j + 1]].x, polygon[indices[j + 1]].y}, vec2{polygon[indices[j + 2]].x, polygon[indices[j + 2]].y}});
	}

	return res;
}
