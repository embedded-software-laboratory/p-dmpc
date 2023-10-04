#include <ArrayTypePrinting.h>
#include <ConfigData.h>
#include <Evaluation.h>
#include <GraphSearchPBIncrementalOptimal.h>
#include <GraphSearchPBOptimal.h>
#include <MPA.h>
#include <MatlabException.h>
#include <MatlabPrinter.h>
#include <PriorityBasedGraphSearch.h>

#include <mex.hpp>
#include <mexAdapter.hpp>

#include "ConfigDataMexFactory.h"
#include "CouplingDataMexFactory.h"
#include "MPAMexFactory.h"
#include "VehicleDataMexFactory.h"
#include "VehicleObstaclesDataMexFactory.h"

struct PriorityBasedResultStructMex {
	matlab::data::CellArray next_nodes;                // {n_hp (1, 8)}
	matlab::data::TypedArray<double> predicted_trims;  // (1, n_hp + 1)
	matlab::data::CellArray y_predicted;               // (1, (tick_per_step + 1) * n_hp, 4)
	matlab::data::CellArray shapes;
	matlab::data::TypedArray<std::uint32_t> n_expanded;  // (1, 1)
	matlab::data::TypedArray<bool> is_exhausted;         // (1, 1)
};

class MexFunction : public matlab::mex::Function /*, private GraphBasedPlanning::Evaluation*/ {
	std::shared_ptr<matlab::engine::MATLABEngine> _matlab = getEngine();
	matlab::data::ArrayFactory _factory;
	std::shared_ptr<GraphBasedPlanning::ConfigData const> _config = nullptr;
	std::shared_ptr<GraphBasedPlanning::MPA const> _mpa = nullptr;

   public:
	MexFunction() /*: GraphBasedPlanning::Evaluation(_config)*/ {
		MatlabPrinter::init(_factory, _matlab);
		Printer::println("MexFunction()");
	}
	~MexFunction() { Printer::println("~MexFunction()"); }

	void operator()(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) final {
		Printer::println("operator()");
		// initialize new graph search:
		if (inputs[0].getType() == matlab::data::ArrayType::ENUM) {
			matlab::data::EnumArray FunctionArray = std::move(inputs[0]);
			if (FunctionArray.getClassName() != "CppOptimizer") throw MatlabException("First Argument must be Enum of type Function! (is ", FunctionArray.getClassName(), ")");
			std::string Function = FunctionArray[0][0];

			if (inputs.size() != 2) throw MatlabException("Wrong number of arguments! (Must be 2, is ", inputs.size(), ")");
			if (inputs[1].getType() != matlab::data::ArrayType::VALUE_OBJECT) throw MatlabException("Data must be VALUE_OBJECT! (is ", inputs[1].getType(), ")");

			if (Function == "InitializeWithScenario") {
				// GraphBasedPlanning::Evaluation::reset();
				_config = std::make_shared<GraphBasedPlanning::ConfigData>(GraphBasedPlanning::make_config(inputs[1], _matlab));
				_mpa = std::make_shared<GraphBasedPlanning::MPA>(GraphBasedPlanning::make_mpa(inputs[1], _config, _matlab));

				Printer::println("Initialized!");
			} else {
				auto [next_nodes_array, predicted_trims_array, y_predicted_array, shapes_array, n_expanded_array, is_exhausted] = run_priority_based(inputs[1], Function);

				outputs[0] = std::move(next_nodes_array);
				outputs[1] = std::move(predicted_trims_array);
				outputs[2] = std::move(y_predicted_array);
				outputs[3] = std::move(shapes_array);
				outputs[4] = std::move(n_expanded_array);
				outputs[5] = std::move(is_exhausted);
			}
		} else {
			throw MatlabException("First Argument must be Enum! (is ", inputs[0].getType(), ")");
		}
	}

   private:
	[[nodiscard]] GraphBasedPlanning::PriorityBasedNode create_root(matlab::data::Array const& iter) const {
		using namespace GraphBasedPlanning;
		matlab::data::TypedArray<double> const trim_indices_array = _matlab->getProperty(iter, u"trim_indices");
		matlab::data::TypedArray<double> const x0_array = _matlab->getProperty(iter, u"x0");
		std::array<std::uint8_t, 1> trim_indices{};
		std::array<double, 1> xs{};
		std::array<double, 1> ys{};
		std::array<double, 1> yaws{};

		for (auto i = 0; i < 1; ++i) {
			trim_indices[i] = trim_indices_array[i] - 1;
			xs[i] = x0_array[i][0];
			ys[i] = x0_array[i][1];
			yaws[i] = x0_array[i][2];
		}

		return {std::move(trim_indices), std::move(xs), std::move(ys), std::move(yaws)};
	}

	template <GraphBasedPlanning::SCENARIO_TYPE scenario_type>
	PriorityBasedResultStructMex run_priority_based(matlab::data::ObjectArray const& iter, std::string const& Function) {
		using namespace GraphBasedPlanning;

		std::array<VehicleData<scenario_type>, 1> vehicle_data = make_vehicle_data<1, scenario_type>(iter, _config, _matlab);
		VehicleObstaclesData vehicle_obstacles_data = make_vehicle_obstacles_data(iter, _config, _matlab);

		PriorityBasedNode root = create_root(iter);

		PriorityBasedGraphSearch<scenario_type>* graph_search = nullptr;

		if (Function == "GraphSearchPBOptimal") {
			graph_search = new GraphSearchPBOptimal<scenario_type>(_config, _mpa, std::move(vehicle_data), std::move(vehicle_obstacles_data));
		} else if (Function == "GraphSearchPBIncrementalOptimal") {
			static GraphSearchPBIncrementalOptimal<scenario_type>* incremental_graph_search = nullptr;

			if (!incremental_graph_search) graph_search = incremental_graph_search = new GraphSearchPBIncrementalOptimal<scenario_type>(_config, _mpa, std::move(vehicle_data), std::move(vehicle_obstacles_data));
			else{
				graph_search = incremental_graph_search;
				incremental_graph_search->update_vehicle_data(vehicle_data);
			}

			while (incremental_graph_search->preperation_done.load() == false)
				;
			while (incremental_graph_search->prep_thread_lock.test_and_set(std::memory_order_acquire))
				;
			incremental_graph_search->preperation_done.store(false);
		} else {
			throw MatlabException("Function '", Function, "' not implemented/available!");
		}

		auto solution = graph_search->search(root);

		std::vector<ColMajorMatrixAccessor<double>> next_nodes(_config->n_hp(), ColMajorMatrixAccessor<double>({1, 8U}));
		ColMajorMatrixAccessor<std::uint8_t> predicted_trims({1, _config->n_hp() + 1});
		std::vector<ColMajorMatrixAccessor<double>> y_predicted(1);
		std::vector<std::vector<std::uint8_t>> y_predicted_trims(1);
		std::vector<std::vector<double>> y_predicted_yaws(1);
		std::vector<std::vector<double>> y_predicted_ys(1);
		std::vector<std::vector<double>> y_predicted_xs(1);
		std::vector<ColMajorMatrixAccessor<double>> shapes(_config->n_hp());
		bool is_exhausted = false;

		if (solution != nullptr) {
			// return {next_nodes, predicted_trims, y_predicted, shapes, is_exhausted};

			for (auto k = _config->n_hp(); k; --k) {  // get the successor node of root of the optimal path
				PriorityBasedNode const* solution_parent;
				if (k > 1) {
					solution_parent = static_cast<PriorityBasedNode const*>(solution->parent());
				} else {
					solution_parent = &root;
				}

				for (auto i = 0; i < 1; ++i) {
					predicted_trims(i, k) = solution->trim(i) + 1U;  // convert to matlab trims

					y_predicted_trims[i].insert(y_predicted_trims[i].begin(), _config->tick_per_step() + 1, solution_parent->trim(i) + 1U);  // convert to matlab trims

					{
						std::vector<double> y_predicted_yaws_tmp = _mpa->yaws(solution_parent->trim(i), solution->trim(i));
						for (auto& e : y_predicted_yaws_tmp) {
							e += solution_parent->yaw(i);
						}
						y_predicted_yaws[i].insert(y_predicted_yaws[i].begin(), y_predicted_yaws_tmp.begin(), y_predicted_yaws_tmp.end());
					}

					{
						auto c = std::cos(solution_parent->yaw(i));
						auto s = std::sin(solution_parent->yaw(i));

						std::vector<double> y_predicted_ys_tmp = _mpa->ys(solution_parent->trim(i), solution->trim(i));
						std::vector<double> y_predicted_xs_tmp = _mpa->xs(solution_parent->trim(i), solution->trim(i));
						for (auto j = 0; j < y_predicted_ys_tmp.size(); ++j) {
							y_predicted_ys_tmp[j] = s * y_predicted_xs_tmp[j] + c * y_predicted_ys_tmp[j] + solution_parent->y(i);
						}
						y_predicted_ys[i].insert(y_predicted_ys[i].begin(), y_predicted_ys_tmp.begin(), y_predicted_ys_tmp.end());

						y_predicted_ys_tmp = _mpa->ys(solution_parent->trim(i), solution->trim(i));
						for (auto j = 0; j < y_predicted_ys_tmp.size(); ++j) {
							y_predicted_xs_tmp[j] = c * y_predicted_xs_tmp[j] - s * y_predicted_ys_tmp[j] + solution_parent->x(i);
						}
						y_predicted_xs[i].insert(y_predicted_xs[i].begin(), y_predicted_xs_tmp.begin(), y_predicted_xs_tmp.end());
					}

					// values see NodeInfo.m

					next_nodes[solution->k() - 1](i, 0) = solution->x(i);
					next_nodes[solution->k() - 1](i, 1) = solution->y(i);
					next_nodes[solution->k() - 1](i, 2) = solution->yaw(i);
					next_nodes[solution->k() - 1](i, 3) = solution->trim(i) + 1.0;  // convert to matlab trims
					next_nodes[solution->k() - 1](i, 4) = solution->g();
					next_nodes[solution->k() - 1](i, 5) = 0.0;  // wrong, but totally unimportant;
					next_nodes[solution->k() - 1](i, 6) = solution->k();

					unsigned size = solution->shapes().size();
					shapes[solution->k() - 1] = ColMajorMatrixAccessor(std::move(solution->shapes()), {2U, size / 2});
				}
				if (k > 1) solution = solution_parent;
			}

			for (auto i = 0; i < 1; ++i) {
				predicted_trims(i, 0) = root.trim(i) + 1U;  // convert to matlab trims

				y_predicted_xs[i].insert(y_predicted_xs[i].end(), y_predicted_ys[i].begin(), y_predicted_ys[i].end());
				y_predicted_xs[i].insert(y_predicted_xs[i].end(), y_predicted_yaws[i].begin(), y_predicted_yaws[i].end());
				y_predicted_xs[i].insert(y_predicted_xs[i].end(), y_predicted_trims[i].begin(), y_predicted_trims[i].end());

				y_predicted[i] = ColMajorMatrixAccessor(std::move(y_predicted_xs[i]), {(_config->tick_per_step() + 1U) * _config->n_hp(), 4U});
			}
		} else {
			is_exhausted = true;
		}

		matlab::data::TypedArray<std::uint32_t> n_expanded_array = _factory.createScalar<std::uint32_t>(std::move(graph_search->get_n_expanded()));

		graph_search->clean();

		matlab::data::TypedArray<bool> is_exhausted_array = _factory.createScalar<bool>(is_exhausted);

		matlab::data::CellArray next_nodes_array = _factory.createCellArray({1, _config->n_hp()});
		for (unsigned i = 0; i < _config->n_hp(); ++i) {
			next_nodes_array[0][i] = _factory.createArray(next_nodes[i].get_dim<std::size_t>(), next_nodes[i].begin(), next_nodes[i].end());
		}

		auto predicted_trims_ = std::vector<double>(predicted_trims.begin(), predicted_trims.end());
		matlab::data::TypedArray<double> predicted_trims_array = _factory.createArray(predicted_trims.get_dim<std::size_t>(), predicted_trims_.begin(), predicted_trims_.end());

		matlab::data::CellArray y_predicted_array = _factory.createCellArray({1, 1});
		for (unsigned int i = 0; i < 1; ++i) {
			y_predicted_array[0][i] = _factory.createArray(y_predicted[i].get_dim<std::size_t>(), y_predicted[i].begin(), y_predicted[i].end());
		}

		matlab::data::CellArray shapes_array = _factory.createCellArray({1, _config->n_hp()});
		for (unsigned int i = 0; i < _config->n_hp(); ++i) {
			shapes_array[0][i] = _factory.createArray(shapes[i].get_dim<std::size_t>(), shapes[i].begin(), shapes[i].end());
		}

		return {next_nodes_array, predicted_trims_array, y_predicted_array, shapes_array, n_expanded_array, is_exhausted_array};  //, n_expanded};
	}

	PriorityBasedResultStructMex run_priority_based(matlab::data::ObjectArray const& iter, std::string const& Function) {
		using namespace GraphBasedPlanning;
		switch (_config->scenario_type()) {
			case SCENARIO_TYPE::Circle: return run_priority_based<SCENARIO_TYPE::Circle>(iter, Function);
			case SCENARIO_TYPE::CommonRoad: return run_priority_based<SCENARIO_TYPE::CommonRoad>(iter, Function);
			default: throw MatlabException("Scenario type '", _config->scenario_type(), "' not implemented/available!");
		}
	}
};