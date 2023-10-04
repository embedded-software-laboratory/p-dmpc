#include "../include/vec2_geometry_operations.h"

double GraphBasedPlanning::trajectory_distance(vec2 const from, unsigned int const index_from, vec2 const to, unsigned int const index_to, std::vector<vec2> const& reference_trajectory_points) {
	if (index_from == index_to) {
		return distance(from, to);
	}

	unsigned int const n_reference_trajectory_line_strips = reference_trajectory_points.size();
	unsigned int current_index = (index_from + 1) % n_reference_trajectory_line_strips;

	double ret = distance(from, reference_trajectory_points[current_index]);

	while (current_index != index_to) {
		unsigned int const next_index = (current_index + 1) % n_reference_trajectory_line_strips;

		ret += distance(reference_trajectory_points[current_index], reference_trajectory_points[next_index]);

		current_index = next_index;
	}

	ret += distance(reference_trajectory_points[index_to], to);

	return ret;
}
