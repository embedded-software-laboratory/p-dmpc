#pragma once

#include <cmath>
#include <cstdio>
#include <limits>
#include <numeric>

#include "earcut.h"

namespace GraphBasedPlanning::CollisionDetection {
	struct vec2 {
		double x;
		double y;
	};
}  // namespace GraphBasedPlanning::CollisionDetection

namespace mapbox::util {
	template <>
	struct nth<0, GraphBasedPlanning::CollisionDetection::vec2> {
		inline static auto get(GraphBasedPlanning::CollisionDetection::vec2 const& t) { return t.x; };
	};
	template <>
	struct nth<1, GraphBasedPlanning::CollisionDetection::vec2> {
		inline static auto get(GraphBasedPlanning::CollisionDetection::vec2 const& t) { return t.y; };
	};
}  // namespace mapbox::util

namespace GraphBasedPlanning::CollisionDetection {
	inline constexpr vec2 add(vec2 a, vec2 b) {
		a.x += b.x;
		a.y += b.y;
		return a;
	}
	inline constexpr vec2 subtract(vec2 a, vec2 b) {
		a.x -= b.x;
		a.y -= b.y;
		return a;
	}
	inline constexpr vec2 negate(vec2 v) {
		v.x = -v.x;
		v.y = -v.y;
		return v;
	}
	inline constexpr vec2 divide(vec2 v, double divisor) {
		v.x /= divisor;
		v.y /= divisor;
		return v;
	}
	inline constexpr vec2 multiply(vec2 v, double multiplier) {
		v.x *= multiplier;
		v.y *= multiplier;
		return v;
	}
	inline constexpr vec2 perpendicular(vec2 v) {
		vec2 p = {v.y, -v.x};
		return p;
	}

	inline constexpr double dotProduct(vec2 a, vec2 b) { return a.x * b.x + a.y * b.y; }
	inline constexpr double lengthSquared(vec2 v) { return v.x * v.x + v.y * v.y; }
	inline constexpr double lengthSquared(vec2 v, vec2 w) { return (v.x - w.x) * (v.x - w.x) + (v.y - w.y) * (v.y - w.y); }
	inline double distance(vec2 v, vec2 w) { return std::sqrt((v.x - w.x) * (v.x - w.x) + (v.y - w.y) * (v.y - w.y)); }
	inline double minimum_distance(vec2 p, vec2 v, vec2 w) {
		// Return minimum distance between line segment vw and point p
		const double l2 = lengthSquared(v, w);  // i.e. |w-v|^2 -  avoid a sqrt
		if (l2 == 0.0) return distance(p, v);   // v == w case
		// Consider the line extending the segment, parameterized as v + t (w - v).
		// We find projection of point p onto the line.
		// It falls where t = [(p-v) . (w-v)] / |w-v|^2
		// We clamp t from [0,1] to handle points outside the segment vw.
		const double t = std::max(0.0, std::min(1.0, dotProduct(subtract(p, v), divide(subtract(w, v), l2))));
		const vec2 projection = add(v, multiply(subtract(w, v), t));  // Projection falls on the segment
		return distance(p, projection);
	}

	inline constexpr double minimum_distance_heuristic(vec2 p, vec2 v, vec2 w) {
		const double l2 = lengthSquared(v, w);
		if (l2 == 0.0) return std::numeric_limits<double>::max();
		const double t = dotProduct(subtract(p, v), divide(subtract(w, v), l2));
		if (t >= 0.0 && t <= 1.0) {
			const vec2 projection = add(v, multiply(subtract(w, v), t));
			return distance(p, projection);
		} else {
			return std::numeric_limits<double>::max();
		}
	}

	// No checks, if in doubt, do not use.
	inline constexpr vec2 minimum_distance_point(vec2 p, vec2 v, vec2 w) {
		const double l2 = lengthSquared(v, w);
		const double t = dotProduct(subtract(p, v), divide(subtract(w, v), l2));
		return add(v, multiply(subtract(w, v), t));
	}

	/*inline constexpr vec2 get_point_along_linestrip_with_distance_from_point(vec2 const p, double distance, vec2 const l1, vec2 const l2) {
	    return vec2;
	}*/

	inline double magnitude(vec2 const v, vec2 const w) { return std::sqrt((v.x * v.x + v.y * v.y) * (w.x * w.x + w.y * w.y)); }

	inline double cosineSimilarity(vec2 const v, vec2 const w) {
		double mag = magnitude(v, w);

		if (mag != 0.0) {
			return dotProduct(v, w) / mag;
		}

		return 0.0;
	}

	inline double getAbsoluteAngleDiff(double const x, double const y) { return M_PI - std::fabs(std::fmod(std::fabs(x - y), 2.0 * M_PI) - M_PI); }

	// helper function for concave collision checks. You can generate triangles out of concave objects to forward them into the collision detection functions.
	inline std::vector<std::array<vec2, 3>> generate_triangles(std::vector<vec2> const& polygon) {
		std::vector<std::array<vec2, 3>> res;

		std::vector<std::vector<vec2>> poly = {polygon};
		std::vector<uint32_t> indices = mapbox::earcut<uint32_t>(poly);

		for (unsigned int j = 0; j < indices.size(); j += 3) {
			res.push_back({vec2{polygon[indices[j]].x, polygon[indices[j]].y}, vec2{polygon[indices[j + 1]].x, polygon[indices[j + 1]].y}, vec2{polygon[indices[j + 2]].x, polygon[indices[j + 2]].y}});
		}

		return res;
	}

	namespace GJK {
		// Copyright (C) 2016 Igor Kroitor <igor.kroitor@gmail.com>.

		//-----------------------------------------------------------------------------
		// Triple product expansion is used to calculate perpendicular normal vectors
		// which kinda 'prefer' pointing towards the Origin in Minkowski space

		inline vec2 tripleProduct(vec2 a, vec2 b, vec2 c) {
			vec2 r;

			double ac = a.x * c.x + a.y * c.y;  // perform a.dot(c)
			double bc = b.x * c.x + b.y * c.y;  // perform b.dot(c)

			// perform b * a.dot(c) - a * b.dot(c)
			r.x = b.x * ac - a.x * bc;
			r.y = b.y * ac - a.y * bc;
			return r;
		}

		//-----------------------------------------------------------------------------
		// This is to compute average center (roughly). It might be different from
		// Center of Gravity, especially for bodies with nonuniform density,
		// but this is ok as initial direction of simplex search in GJK.

		inline vec2 averagePoint(const vec2* vertices, size_t count) {
			vec2 avg = {0.f, 0.f};
			for (size_t i = 0; i < count; i++) {
				avg.x += vertices[i].x;
				avg.y += vertices[i].y;
			}
			avg.x /= count;
			avg.y /= count;
			return avg;
		}

		//-----------------------------------------------------------------------------
		// Get furthest vertex along a certain direction

		inline size_t indexOfFurthestPoint(const vec2* vertices, size_t count, vec2 d) {
			double maxProduct = dotProduct(d, vertices[0]);
			size_t index = 0;
			for (size_t i = 1; i < count; i++) {
				double product = dotProduct(d, vertices[i]);
				if (product > maxProduct) {
					maxProduct = product;
					index = i;
				}
			}
			return index;
		}

		//-----------------------------------------------------------------------------
		// Minkowski sum support function for GJK

		inline vec2 support(const vec2* vertices1, size_t count1, const vec2* vertices2, size_t count2, vec2 d) {
			// get furthest point of first body along an arbitrary direction
			size_t i = indexOfFurthestPoint(vertices1, count1, d);

			// get furthest point of second body along the opposite direction
			size_t j = indexOfFurthestPoint(vertices2, count2, negate(d));

			// subtract (Minkowski sum) the two points to see if bodies 'overlap'
			return subtract(vertices1[i], vertices2[j]);
		}

		//-----------------------------------------------------------------------------
		// The GJK yes/no test

		// Input should be a pointer to a vec2 data field and its size.
		// For matrix data in ColMajor format of type double the size must be adapted (divided by 2).
		inline bool gjk(const vec2* vertices1, size_t count1, const vec2* vertices2, size_t count2) {
			size_t index = 0;  // index of current vertex of simplex
			vec2 a, b, c, d, ao, ab, ac, abperp, acperp, simplex[3];

			vec2 position1 = averagePoint(vertices1, count1);  // not a CoG but
			vec2 position2 = averagePoint(vertices2, count2);  // it's ok for GJK )

			// initial direction from the center of 1st body to the center of 2nd body
			d = subtract(position1, position2);

			// if initial direction is zero – set it to any arbitrary axis (we choose X)
			if ((d.x == 0) && (d.y == 0)) d.x = 1.f;

			// set the first support as initial point of the new simplex
			a = simplex[0] = support(vertices1, count1, vertices2, count2, d);

			if (dotProduct(a, d) <= 0) return false;  // no collision

			d = negate(a);  // The next search direction is always towards the origin, so the next search direction is negate(a)

			while (true) {
				a = simplex[++index] = support(vertices1, count1, vertices2, count2, d);

				if (dotProduct(a, d) <= 0) return false;  // no collision

				ao = negate(a);  // from point A to Origin is just negative A

				// simplex has 2 points (a line segment, not a triangle yet)
				if (index < 2) {
					b = simplex[0];
					ab = subtract(b, a);            // from point A to B
					d = tripleProduct(ab, ao, ab);  // normal to AB towards Origin
					if (lengthSquared(d) == 0) d = perpendicular(ab);
					continue;  // skip to next iteration
				}

				b = simplex[1];
				c = simplex[0];
				ab = subtract(b, a);  // from point A to B
				ac = subtract(c, a);  // from point A to C

				acperp = tripleProduct(ab, ac, ac);

				if (dotProduct(acperp, ao) >= 0) {
					d = acperp;  // new direction is normal to AC towards Origin

				} else {
					abperp = tripleProduct(ac, ab, ab);

					if (dotProduct(abperp, ao) < 0) return true;  // collision

					simplex[0] = simplex[1];  // swap first element (point C)

					d = abperp;  // new direction is normal to AB towards Origin
				}

				simplex[1] = simplex[2];  // swap element in the middle (point B)
				--index;
			}

			return false;
		}

		// to check collision for concave objects, you have to first generate triangles with the ear-cut algorithm. There is a helper function 'generate_triangles'.
		inline bool check_collision_concave(std::vector<std::array<vec2, 3>> const& triangles1, std::vector<std::array<vec2, 3>> const& triangles2) {
			for (auto const& k : triangles1) {
				for (auto const& l : triangles2) {
					if (gjk(k.data(), k.size(), l.data(), l.size())) return true;
				}
			}

			return false;
		}

	}  // namespace GJK

	namespace SAT {
		// Input should be a pointer to a vec2 data field and its size.
		// For matrix data in ColMajor format of type double the size must be adapted (divided by 2).
		inline bool sat(vec2 const* const poly1, unsigned int const count1, vec2 const* const poly2, unsigned int const count2) {
			for (unsigned int i = 0; i < count1; ++i) {
				unsigned int j = (i + 1) % count1;
				vec2 axis = {-(poly1[j].y - poly1[i].y), poly1[j].x - poly1[i].x};
				double norm = std::sqrt(axis.x * axis.x + axis.y * axis.y);
				axis = {axis.x / norm, axis.y / norm};

				double min_poly1 = std::numeric_limits<double>::infinity(), max_poly1 = -std::numeric_limits<double>::infinity();
				for (unsigned int k = 0; k < count1; ++k) {
					double dot = (poly1[k].x * axis.x + poly1[k].y * axis.y);
					min_poly1 = std::min(min_poly1, dot);
					max_poly1 = std::max(max_poly1, dot);
				}

				double min_poly2 = std::numeric_limits<double>::infinity(), max_poly2 = -std::numeric_limits<double>::infinity();
				for (unsigned int k = 0; k < count2; ++k) {
					double dot = (poly2[k].x * axis.x + poly2[k].y * axis.y);
					min_poly2 = std::min(min_poly2, dot);
					max_poly2 = std::max(max_poly2, dot);
				}

				if (!(max_poly2 >= min_poly1 && max_poly1 >= min_poly2)) return false;
			}

			for (unsigned int i = 0; i < count2; ++i) {
				unsigned int j = (i + 1) % count2;
				vec2 axis = {-(poly2[j].y - poly2[i].y), poly2[j].x - poly2[i].x};
				double norm = std::sqrt(axis.x * axis.x + axis.y * axis.y);
				axis = {axis.x / norm, axis.y / norm};

				double min_poly1 = std::numeric_limits<double>::infinity(), max_poly1 = -std::numeric_limits<double>::infinity();
				for (unsigned int k = 0; k < count1; ++k) {
					double dot = (poly1[k].x * axis.x + poly1[k].y * axis.y);
					min_poly1 = std::min(min_poly1, dot);
					max_poly1 = std::max(max_poly1, dot);
				}

				double min_poly2 = std::numeric_limits<double>::infinity(), max_poly2 = -std::numeric_limits<double>::infinity();
				for (unsigned int k = 0; k < count2; ++k) {
					double dot = (poly2[k].x * axis.x + poly2[k].y * axis.y);
					min_poly2 = std::min(min_poly2, dot);
					max_poly2 = std::max(max_poly2, dot);
				}

				if (!(max_poly2 >= min_poly1 && max_poly1 >= min_poly2)) return false;
			}

			return true;
		}

		// to check collision for concave objects, you have to first generate triangles with the ear-cut algorithm. There is a helper function 'generate_triangles'.
		inline bool check_collision_concave(std::vector<std::array<vec2, 3>> const& triangles1, std::vector<std::array<vec2, 3>> const& triangles2) {
			for (const auto& k : triangles1) {
				for (const auto& l : triangles2) {
					if (sat(k.data(), k.size(), l.data(), l.size())) return true;
				}
			}

			return false;
		}
	}  // namespace SAT
}  // namespace GraphBasedPlanning::CollisionDetection
