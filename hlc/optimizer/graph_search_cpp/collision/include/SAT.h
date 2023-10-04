#pragma once

#include "CollisionDetection.h"

class SAT : public CollisionDetection {
   public:
	[[nodiscard]] bool check_collision(vec2 const* poly1, unsigned int count1, vec2 const* poly2, unsigned int count2) const final;
};