#include <Node.h>
#include <Printer.h>
#include <vec2.h>

#include <memory>
#include <vector>

#include "ConfigData.h"
#include "Eval.h"
#include "ScenarioType.h"
#include "vec2_boost_adapter.h"
#include "vec2_geometry_operations.h"

namespace GraphBasedPlanning {
	struct EvalData {
		std::vector<vec2> old_positions_on_trajectory;
		std::vector<double> cumsum_distance_of_trajectory_travelled;
		std::vector<int> index_on_trajectory;
	};

	class Evaluation {
		std::vector<EvalData> _eval_data;
		std::shared_ptr<GraphBasedPlanning::ConfigData const> const& _config;

	   protected:
		Evaluation(std::shared_ptr<GraphBasedPlanning::ConfigData const> const& config) : _config(config) {}
		void reset() { _eval_data.clear(); }
		EvalData& last() { return _eval_data.back(); }

		template <unsigned int n_vehicles, GraphBasedPlanning::SCENARIO_TYPE scenario_type>
		void evaluate(GraphBasedPlanning::Node<n_vehicles> const* const solution, GraphBasedPlanning::Node<n_vehicles> const root) {
			using namespace GraphBasedPlanning;

			if (_eval_data.empty()) {
				EvalData eval_data;

				for (auto i = 0; i < n_vehicles; ++i) {
					eval_data.old_positions_on_trajectory.emplace_back(root.x(i), root.y(i));
					eval_data.cumsum_distance_of_trajectory_travelled.push_back(0.0);
					eval_data.index_on_trajectory.push_back(0);
				}
				_eval_data.push_back(eval_data);
			}

			EvalData eval_data;

			std::vector<double> distances;

			for (auto i = 0; i < n_vehicles; ++i) {
				Printer::print(_eval_data.back().index_on_trajectory[i], "->");
				auto [new_position, new_index] = closest_point<scenario_type, 5, 5>({solution->x(i), solution->y(i)}, _config->reference_trajectory()[i], _eval_data.back().index_on_trajectory[i]);
				Printer::print(new_index);

				eval_data.old_positions_on_trajectory.push_back(new_position);
				eval_data.index_on_trajectory.push_back(new_index);

				double distance = trajectory_distance(
				    _eval_data.back().old_positions_on_trajectory[i], _eval_data.back().index_on_trajectory[i], eval_data.old_positions_on_trajectory[i], eval_data.index_on_trajectory[i], _config->reference_trajectory()[i]);

				distances.push_back(distance);

				eval_data.cumsum_distance_of_trajectory_travelled.push_back(_eval_data.back().cumsum_distance_of_trajectory_travelled[i] + distance);
				Printer::println(", ", eval_data.cumsum_distance_of_trajectory_travelled[i]);
			}
			_eval_data.push_back(eval_data);

#if DO_EVAL
			save(distance_travelled_along_trajectory, distances);
			save(average_distance_travelled_along_trajectory, std::accumulate(distances.begin(), distances.end(), 0.0) / static_cast<double>(distances.size()));
#endif
			std::vector<double> distances_to_goal;
			double added_distances_to_goal = 0.0;

			if (scenario_type == SCENARIO_TYPE::Circle) {
				for (auto i = 0; i < n_vehicles; ++i) {
					distances_to_goal.push_back(distance({solution->x(i), solution->y(i)}, _config->reference_trajectory()[i][1]));

					Printer::println("Distance: ", distances_to_goal.back());
					added_distances_to_goal += distances_to_goal.back();
				}

				Printer::println("Distance added: ", added_distances_to_goal);
			}
		}
	};
}  // namespace GraphBasedPlanning