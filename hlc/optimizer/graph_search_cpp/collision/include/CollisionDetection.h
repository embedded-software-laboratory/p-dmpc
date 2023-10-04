#pragma once

#include "vec2.h"

#include <vector>
#include <cstdint>
#include <array>

class CollisionDetection {
   public:
	[[nodiscard]] virtual bool check_collision(vec2 const* poly1, unsigned int count1, vec2 const* poly2, unsigned int count2) const = 0;

	// to check collision for concave objects, you have to first generate triangles with the ear-cut algorithm. There is a helper function 'generate_triangles'.
	[[nodiscard]] bool check_collision_concave(std::vector<std::array<vec2, 3>> const& triangles1, std::vector<std::array<vec2, 3>> const& triangles2) const;

	// helper function for concave collision checks. You can generate triangles out of concave objects to forward them into the collision detection functions.
	[[nodiscard]] static std::vector<std::array<vec2, 3>> generate_triangles(std::vector<vec2> const& polygon);
};