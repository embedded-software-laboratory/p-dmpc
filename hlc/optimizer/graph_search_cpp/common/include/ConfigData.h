#pragma once

#include <Node.h>
#include <vec2.h>

#include <vector>

#include "ScenarioType.h"

namespace GraphBasedPlanning {
	class ConfigData {
	   private:
		unsigned int _n_hp = 0;
		unsigned int _n_vehicles = 0;
		unsigned int _tick_per_step = 0;
		unsigned int _n_trims = 0;
		double _dt = 0.0;
		SCENARIO_TYPE _scenario_type = SCENARIO_TYPE::Error;
		std::vector<std::vector<vec2>> _reference_trajectory;
		bool _recursive_feasibility = true;

	   public:
		ConfigData(unsigned int n_hp, unsigned int n_vehicles, unsigned int n_trims, unsigned int tick_per_step, double dt, SCENARIO_TYPE scenario_type, std::vector<std::vector<vec2>> const&& reference_trajectory, bool recursive_feasibility);

		[[nodiscard]] unsigned int n_hp() const { return _n_hp; }
		[[nodiscard]] unsigned int n_vehicles() const { return _n_vehicles; }
		[[nodiscard]] unsigned int n_trims() const { return _n_trims; }
		[[nodiscard]] unsigned int tick_per_step() const { return _tick_per_step; }
		[[nodiscard]] double dt() const { return _dt; }
		[[nodiscard]] SCENARIO_TYPE scenario_type() const { return _scenario_type; }
		[[nodiscard]] std::vector<std::vector<vec2>> const& reference_trajectory() const { return _reference_trajectory; }
		[[nodiscard]] bool recursive_feasibility() const { return _recursive_feasibility; }
	};
}  // namespace GraphBasedPlanning