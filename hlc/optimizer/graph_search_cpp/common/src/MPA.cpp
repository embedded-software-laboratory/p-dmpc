#include "../include/MPA.h"

GraphBasedPlanning::MPA::MPA(std::vector<std::vector<std::vector<std::uint8_t>>> const reachability_list, std::vector<std::vector<std::vector<std::uint8_t>>> const transition_matrix_single,
    std::vector<std::vector<double>> const&& transition_matrix_mean_speed, std::vector<std::vector<Maneuver<double>>> const&& maneuvers, std::vector<std::uint16_t> const&& distance_to_equilibrium)
    : _reachability_list(reachability_list), _transition_matrix_single(transition_matrix_single), _transition_matrix_mean_speed(transition_matrix_mean_speed), _maneuvers(maneuvers), _distance_to_equilibrium(distance_to_equilibrium) {
	std::vector<double> max_speeds(_transition_matrix_mean_speed.size());
	for (auto i = 0; i < _transition_matrix_mean_speed.size(); ++i) max_speeds[i] = *std::max_element(_transition_matrix_mean_speed[i].begin(), _transition_matrix_mean_speed[i].end());
	_max_speed = *std::max_element(max_speeds.begin(), max_speeds.end());
}