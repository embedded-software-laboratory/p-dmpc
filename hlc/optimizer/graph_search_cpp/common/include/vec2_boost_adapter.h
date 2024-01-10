#pragma once

#include <vec2.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>
BOOST_GEOMETRY_REGISTER_POINT_2D(vec2, double, boost::geometry::cs::cartesian, x, y)

#include <boost/geometry/geometries/register/linestring.hpp>
#include <vector>
BOOST_GEOMETRY_REGISTER_LINESTRING(std::vector<vec2>)