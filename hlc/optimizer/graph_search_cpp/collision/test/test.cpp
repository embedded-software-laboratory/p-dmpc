#include <SAT.h>
#include <matplotlibcpp.h>
#include <vec2.h>

#include <algorithm>
#include <array>
#include <iostream>
#include <random>
#include <vector>

std::array<vec2, 2> generate_random_line() {
	thread_local static std::random_device rd;
	thread_local static std::mt19937 mt{rd()};
	thread_local static std::uniform_real_distribution<double> uniform_real_distribution;

	return {vec2{uniform_real_distribution(mt), uniform_real_distribution(mt)}, vec2{uniform_real_distribution(mt), uniform_real_distribution(mt)}};
}

std::vector<vec2> generate_random_convex_polygon(unsigned int n) {
	thread_local static std::random_device rd;
	thread_local static std::mt19937 mt{rd()};
	thread_local static std::uniform_int_distribution<> uniform_bool_distribution(0, 1);
	thread_local static std::uniform_real_distribution<double> uniform_real_distribution;

	std::vector<double> x_pool;
	x_pool.reserve(n);
	std::vector<double> y_pool;
	y_pool.reserve(n);

	for (int i = 0; i < n; i++) {
		x_pool.push_back(uniform_real_distribution(mt));
		y_pool.push_back(uniform_real_distribution(mt));
	}

	std::sort(x_pool.begin(), x_pool.end());
	std::sort(y_pool.begin(), y_pool.end());

	double min_x = x_pool[0];
	double max_x = x_pool[n - 1];
	double min_y = y_pool[0];
	double max_y = y_pool[n - 1];

	std::vector<double> x_vec;
	x_vec.reserve(n);
	std::vector<double> y_vec;
	y_vec.reserve(n);

	double last_top = min_x, lastBot = min_x;

	for (int i = 1; i < n - 1; i++) {
		double x = x_pool[i];

		if (uniform_bool_distribution(mt)) {
			x_vec.push_back(x - last_top);
			last_top = x;
		} else {
			x_vec.push_back(lastBot - x);
			lastBot = x;
		}
	}

	x_vec.push_back(max_x - last_top);
	x_vec.push_back(lastBot - max_x);

	double last_left = min_y, lastRight = min_y;

	for (int i = 1; i < n - 1; i++) {
		double y = y_pool[i];

		if (uniform_bool_distribution(mt)) {
			y_vec.push_back(y - last_left);
			last_left = y;
		} else {
			y_vec.push_back(lastRight - y);
			lastRight = y;
		}
	}

	y_vec.push_back(max_y - last_left);
	y_vec.push_back(lastRight - max_y);

	std::shuffle(y_vec.begin(), y_vec.end(), mt);

	std::vector<vec2> vec;
	vec.reserve(n);

	for (int i = 0; i < n; i++) {
		vec.push_back(vec2{x_vec[i], y_vec[i]});
	}

	std::sort(vec.begin(), vec.end(), [](vec2 const lhs, vec2 const rhs) { return std::atan2(lhs.y, lhs.x) < std::atan2(rhs.y, rhs.x); });

	double x = 0, y = 0;
	double min_polygon_x = 0;
	double min_polygon_y = 0;
	std::vector<vec2> points;
	points.reserve(n);

	for (int i = 0; i < n; i++) {
		points.push_back(vec2{x, y});

		x += vec[i].x;
		y += vec[i].y;

		min_polygon_x = std::min(min_polygon_x, x);
		min_polygon_y = std::min(min_polygon_y, y);
	}

	double x_shift = min_x - min_polygon_x;
	double y_shift = min_y - min_polygon_y;

	for (int i = 0; i < n; i++) {
		points[i].x += x_shift;
		points[i].y += y_shift;
	}

	return points;
}

std::vector<vec2> generate_random_polygon(unsigned int n) {
	thread_local static std::random_device rd;
	thread_local static std::mt19937 mt{rd()};
	thread_local static std::uniform_real_distribution<double> uniform_real_distribution;

	std::vector<vec2> points(n);

	double sum_x = 0.0;
	double sum_y = 0.0;

	for (int i = 0; i < n; ++i) {
		sum_x += points[i].x = uniform_real_distribution(mt);
		sum_y += points[i].y = uniform_real_distribution(mt);
	}

	double mean_x = sum_x / n;
	double mean_y = sum_y / n;

	std::sort(points.begin(), points.end(), [mean_x, mean_y](vec2 const lhs, vec2 const rhs) { return std::atan2(lhs.y - mean_y, lhs.x - mean_x) < std::atan2(rhs.y - mean_y, rhs.x - mean_x); });

	return points;
}

std::tuple<std::vector<double>, std::vector<double>> convert(std::vector<vec2> const& vertices) {
	std::vector<double> xs(vertices.size() + 1);
	std::vector<double> ys(vertices.size() + 1);

	for (auto i = 0; i < vertices.size(); ++i) {
		xs[i] = vertices[i].x;
		ys[i] = vertices[i].y;
	}

	xs.back() = vertices.front().x;
	ys.back() = vertices.front().y;

	return {xs, ys};
}

std::tuple<std::vector<double>, std::vector<double>> convert(std::array<vec2, 2> const& line) {
	std::vector<double> xs(line.size());
	std::vector<double> ys(line.size());

	for (auto i = 0; i < line.size(); ++i) {
		xs[i] = line[i].x;
		ys[i] = line[i].y;
	}

	return {xs, ys};
}

int main() {
	static SAT sat;

	std::vector<std::vector<vec2>> vertices = {generate_random_convex_polygon(5), generate_random_convex_polygon(5)};
	std::vector<std::array<vec2, 2>> lines = {generate_random_line()};

	for (unsigned int i = 0; i < vertices.size(); ++i) {
		for (unsigned int j = i + 1; j < vertices.size(); ++j) {
			std::cout << "SAT: The polygons intersect " << std::boolalpha << sat.check_collision(vertices[i].data(), vertices[i].size(), vertices[j].data(), vertices[j].size()) << std::endl;
		}
	}

	for (unsigned int i = 0; i < vertices.size(); ++i) {
		for (unsigned int j = 0; j < lines.size(); ++j) {
			std::cout << "SAT_LINE: The polygon an line intersect " << std::boolalpha << sat.check_collision(vertices[i].data(), vertices[i].size(), lines[j].data(), lines[j].size()) << std::endl;
		}
	}

	auto [poly1_xs, poly1_ys] = convert(vertices[0]);
	auto [poly2_xs, poly2_ys] = convert(vertices[1]);
	auto [line_xs, line_ys] = convert(lines[0]);

	namespace plt = matplotlibcpp;

	plt::plot(poly1_xs, poly1_ys);
	plt::plot(poly2_xs, poly2_ys);
	plt::plot(line_xs, line_ys);

	plt::show();
}