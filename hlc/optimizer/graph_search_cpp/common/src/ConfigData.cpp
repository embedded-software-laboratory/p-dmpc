#include "../include/ConfigData.h"

GraphBasedPlanning::ConfigData::ConfigData(const unsigned int n_hp, const unsigned int n_vehicles, const unsigned int n_trims, const unsigned int tick_per_step, const double dt, const GraphBasedPlanning::SCENARIO_TYPE scenario_type,
    const std::vector<std::vector<vec2>>&& reference_trajectory, bool recursive_feasibility)
    : _n_hp(n_hp), _n_vehicles(n_vehicles), _n_trims(n_trims), _tick_per_step(tick_per_step), _dt(dt), _scenario_type(scenario_type), _reference_trajectory(reference_trajectory), _recursive_feasibility(recursive_feasibility) {}