#include <ArrayTypePrinting.h>

#include "../../config.h"

// must come before this
#include <ConfigData.h>
#include <Evaluation.h>
#include <MPA.h>
#include <MatlabException.h>
#include <MatlabPrinter.h>

#include <mex.hpp>
#include <mexAdapter.hpp>
#include <set>

#include "CentralizedConflictBasedLiterature.h"
#include "CentralizedGrouping.h"
#include "CentralizedGroupingConflictBased.h"
#include "CentralizedIncremental.h"
#include "CentralizedNaiveMonteCarloPolymorphic.h"
#include "CentralizedNaiveMonteCarloPolymorphicParallel.h"
#include "CentralizedNaiveMonteCarloSimple.h"
#include "CentralizedOptimalLeafParallelization.h"
#include "CentralizedOptimalPolymorphic.h"
#include "CentralizedOptimalRootParallelization.h"
#include "CentralizedOptimalSimple.h"
#include "ConfigDataMexFactory.h"
#include "CouplingDataMexFactory.h"
#include "MPAMexFactory.h"
#include "VehicleDataMexFactory.h"

struct CentralizedResultStructMex {
	matlab::data::CellArray next_nodes;                // {n_hp (n_vehicles, 8)}
	matlab::data::TypedArray<double> predicted_trims;  // (n_vehicles, n_hp + 1)
	matlab::data::CellArray y_predicted;               // (n_vehicles, (tick_per_step + 1) * n_hp, 4)
	                                                   // matlab::data::TypedArray<double> n_expanded;       // (1, 1)
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
		// initialize new graph search:
		if (inputs[0].getType() == matlab::data::ArrayType::ENUM) {
			matlab::data::EnumArray FunctionArray = std::move(inputs[0]);
			if (FunctionArray.getClassName() != "CppOptimizer") throw MatlabException("First Argument must be Enum of type Function! (is ", FunctionArray.getClassName(), ")");
			std::string Function = FunctionArray[0][0];

			if (inputs.size() < 2) throw MatlabException("Wrong number of arguments! (Must be 2 or more, is ", inputs.size(), ")");
			if (inputs[1].getType() != matlab::data::ArrayType::VALUE_OBJECT) throw MatlabException("Data must be VALUE_OBJECT! (is ", inputs[1].getType(), ")");

			if (Function == "InitializeWithScenario") {
				if (inputs.size() != 4) throw MatlabException("Wrong number of arguments! (Must be 4, is ", inputs.size(), ")");
				// GraphBasedPlanning::Evaluation::reset();
				_config = std::make_shared<GraphBasedPlanning::ConfigData>(GraphBasedPlanning::make_config(inputs[1], inputs[2], inputs[3], _matlab));
				_mpa = std::make_shared<GraphBasedPlanning::MPA>(GraphBasedPlanning::make_mpa(inputs[2], _config, _matlab));

				Printer::println("Initialized!");
			} else {
#if DO_EVAL
				EvaluationDataSaver::Instance().start_iteration();
#endif
				if (inputs.size() != 2) throw MatlabException("Wrong number of arguments! (Must be 2, is ", inputs.size(), ")");
				auto [next_nodes_array, predicted_trims_array, y_predicted_array /*, n_expanded*/] = run_centralized(inputs[1], Function);

				outputs[0] = std::move(next_nodes_array);
				outputs[1] = std::move(predicted_trims_array);
				outputs[2] = std::move(y_predicted_array);
#if DO_EVAL
				EvaluationDataSaver::Instance().end_iteration();
#endif
			}
		} else {
			throw MatlabException("First Argument must be Enum! (is ", inputs[0].getType(), ")");
		}
	}

   private:
	template <unsigned int n_vehicles>
	[[nodiscard]] GraphBasedPlanning::Node<n_vehicles> create_root(matlab::data::Array const& iter) const {
		using namespace GraphBasedPlanning;
		matlab::data::TypedArray<double> const trim_indices_array = _matlab->getProperty(iter, u"trim_indices");
		matlab::data::TypedArray<double> const x0_array = _matlab->getProperty(iter, u"x0");
		std::array<std::uint8_t, n_vehicles> trim_indices{};
		std::array<double, n_vehicles> xs{};
		std::array<double, n_vehicles> ys{};
		std::array<double, n_vehicles> yaws{};

		for (auto i = 0; i < n_vehicles; ++i) {
			trim_indices[i] = trim_indices_array[i] - 1;
			xs[i] = x0_array[i][0];
			ys[i] = x0_array[i][1];
			yaws[i] = x0_array[i][2];
		}

		return Node<n_vehicles>(std::move(trim_indices), std::move(xs), std::move(ys), std::move(yaws));
	}

	template <unsigned int n_vehicles, GraphBasedPlanning::SCENARIO_TYPE scenario_type>
	CentralizedResultStructMex run_centralized(matlab::data::ObjectArray const& iter, std::string const& Function) {
		using namespace GraphBasedPlanning;

		std::array<VehicleData<scenario_type>, n_vehicles> vehicle_data = make_vehicle_data<n_vehicles, scenario_type>(iter, _config, _matlab);
		Node<n_vehicles> root = create_root<n_vehicles>(iter);

		CentralizedGraphSearch<n_vehicles, scenario_type>* graph_search = nullptr;
#if DO_EVAL
		save(function_used, Function);
#endif
		if (Function == "CentralizedOptimal") {
			graph_search = new CentralizedOptimalSimple<n_vehicles, scenario_type, false>(_config, _mpa, std::move(vehicle_data));
		} else if (Function == "CentralizedOptimalPolymorphic") {
			graph_search = new CentralizedOptimalPolymorphic<n_vehicles, scenario_type>(_config, _mpa, std::move(vehicle_data));
		} else if (Function == "CentralizedOptimalMemorySaving") {
			graph_search = new CentralizedOptimalSimple<n_vehicles, scenario_type, true>(_config, _mpa, std::move(vehicle_data));
		} else if (Function == "CentralizedOptimalAStarParallelization") {
			graph_search = new CentralizedOptimalRootParallelization<n_vehicles, scenario_type, Threads>(_config, _mpa, std::move(vehicle_data));
		} else if (Function == "CentralizedOptimalNodeParallelization") {
			graph_search = new CentralizedOptimalLeafParallelization<n_vehicles, scenario_type, Threads>(_config, _mpa, std::move(vehicle_data));
		} else if (Function == "CentralizedNaiveMonteCarlo") {
			graph_search = new CentralizedNaiveMonteCarloSimple<n_vehicles, scenario_type, Experiments>(_config, _mpa, std::move(vehicle_data));
		} else if (Function == "CentralizedNaiveMonteCarloPolymorphic") {
			graph_search = new CentralizedNaiveMonteCarloPolymorphic<n_vehicles, scenario_type, Experiments>(_config, _mpa, std::move(vehicle_data));
		} else if (Function == "CentralizedNaiveMonteCarloPolymorphicParallel") {
			graph_search = new CentralizedNaiveMonteCarloPolymorphicParallel<n_vehicles, scenario_type, Experiments, Threads>(_config, _mpa, std::move(vehicle_data));
		} else if (Function == "CentralizedOptimalIncremental") {
			static CentralizedIncremental<n_vehicles, scenario_type>* incremental_graph_search = nullptr;

			using namespace std::chrono_literals;
			static Watchdog dog(5s, [] {
				delete incremental_graph_search;
				incremental_graph_search = nullptr;
			});

			dog.stop();

			IncrementalNode<n_vehicles>::n_hp(_config->n_hp());

			if (incremental_graph_search) {
				incremental_graph_search->update_vehicle_data(vehicle_data);
				graph_search = incremental_graph_search;
			} else {
				graph_search = incremental_graph_search = new CentralizedIncremental<n_vehicles, scenario_type>(_config, _mpa, std::move(vehicle_data), dog);
			}
		} else if (Function == "CentralizedConflictBased") {
			graph_search = new CentralizedConflictBasedLiterature<n_vehicles, scenario_type>(_config, _mpa, std::move(vehicle_data));
		} else if (Function == "CentralizedGrouping") {
			CouplingData coupling_data = make_coupling_data(iter, _config, _matlab);
			graph_search = new CentralizedGrouping<n_vehicles, scenario_type>(_config, _mpa, std::move(vehicle_data), coupling_data);
		} else {
			throw MatlabException("Function '", Function, "' not implemented/available!");
		}

		auto t0 = std::chrono::high_resolution_clock::now();

		auto solution = graph_search->search(root);

		auto t1 = std::chrono::high_resolution_clock::now();
		auto elapsed = t1 - t0;
		auto seconds = std::chrono::duration<double>(elapsed).count();
#if DO_EVAL
		save(time_used, seconds);
		save(g_value, solution->g());
#endif

		std::vector<ColMajorMatrixAccessor<double>> next_nodes(_config->n_hp(), ColMajorMatrixAccessor<double>({_config->n_vehicles(), 8U}));
		ColMajorMatrixAccessor<std::uint8_t> predicted_trims({n_vehicles, _config->n_hp() + 1});
		std::vector<ColMajorMatrixAccessor<double>> y_predicted(n_vehicles);
		std::vector<std::vector<std::uint8_t>> y_predicted_trims(n_vehicles);
		std::vector<std::vector<double>> y_predicted_yaws(n_vehicles);
		std::vector<std::vector<double>> y_predicted_ys(n_vehicles);
		std::vector<std::vector<double>> y_predicted_xs(n_vehicles);

		for (auto k = _config->n_hp(); k; --k) {  // get the successor node of root of the optimal path
			Node<n_vehicles> const* solution_parent;
			if (k > 1) {
				solution_parent = solution->parent();
			} else {
				solution_parent = &root;
			}

			for (auto i = 0; i < n_vehicles; ++i) {
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
				if (Function == "CentralizedOptimalIncremental") {
					next_nodes[k - 1](i, 0) = solution->x(i);
					next_nodes[k - 1](i, 1) = solution->y(i);
					next_nodes[k - 1](i, 2) = solution->yaw(i);
					next_nodes[k - 1](i, 3) = solution->trim(i) + 1.0;  // convert to matlab trims
					next_nodes[k - 1](i, 4) = solution->g();
					next_nodes[k - 1](i, 5) = 0.0;  // wrong, but totally unimportant;
					next_nodes[k - 1](i, 6) = solution->k();
				} else {
					next_nodes[solution->k() - 1](i, 0) = solution->x(i);
					next_nodes[solution->k() - 1](i, 1) = solution->y(i);
					next_nodes[solution->k() - 1](i, 2) = solution->yaw(i);
					next_nodes[solution->k() - 1](i, 3) = solution->trim(i) + 1.0;  // convert to matlab trims
					next_nodes[solution->k() - 1](i, 4) = solution->g();
					next_nodes[solution->k() - 1](i, 5) = 0.0;  // wrong, but totally unimportant;
					next_nodes[solution->k() - 1](i, 6) = solution->k();
				}
			}
			if (k > 1) solution = solution_parent;
		}

		// Evaluation::evaluate<n_vehicles, scenario_type>(solution, root);

		for (auto i = 0; i < n_vehicles; ++i) {
			predicted_trims(i, 0) = root.trim(i) + 1U;  // convert to matlab trims

			y_predicted_xs[i].insert(y_predicted_xs[i].end(), y_predicted_ys[i].begin(), y_predicted_ys[i].end());
			y_predicted_xs[i].insert(y_predicted_xs[i].end(), y_predicted_yaws[i].begin(), y_predicted_yaws[i].end());
			y_predicted_xs[i].insert(y_predicted_xs[i].end(), y_predicted_trims[i].begin(), y_predicted_trims[i].end());

			y_predicted[i] = ColMajorMatrixAccessor(std::move(y_predicted_xs[i]), {(_config->tick_per_step() + 1U) * _config->n_hp(), 4U});
		}

		graph_search->clean();

		matlab::data::CellArray next_nodes_array = _factory.createCellArray({1, _config->n_hp()});
		for (unsigned i = 0; i < _config->n_hp(); ++i) {
			next_nodes_array[0][i] = _factory.createArray(next_nodes[i].get_dim<std::size_t>(), next_nodes[i].begin(), next_nodes[i].end());
		}

		auto predicted_trims_ = std::vector<double>(predicted_trims.begin(), predicted_trims.end());
		matlab::data::TypedArray<double> predicted_trims_array = _factory.createArray(predicted_trims.get_dim<std::size_t>(), predicted_trims_.begin(), predicted_trims_.end());

		matlab::data::CellArray y_predicted_array = _factory.createCellArray({1, n_vehicles});
		for (unsigned int i = 0; i < n_vehicles; ++i) {
			y_predicted_array[0][i] = _factory.createArray(y_predicted[i].get_dim<std::size_t>(), y_predicted[i].begin(), y_predicted[i].end());
		}

		// matlab::data::TypedArray<double> n_expanded = _factory.createArray({1, 1}, {0.0});

		return {next_nodes_array, predicted_trims_array, y_predicted_array};  //, n_expanded};
	}

	template <GraphBasedPlanning::SCENARIO_TYPE scenario_type>
	CentralizedResultStructMex run_centralized(matlab::data::ObjectArray const& iter, std::string const& Function) {
		using namespace GraphBasedPlanning;
		switch (_config->n_vehicles()) {
			case 1: return run_centralized<1, scenario_type>(iter, Function);
			case 2: return run_centralized<2, scenario_type>(iter, Function);
			case 3: return run_centralized<3, scenario_type>(iter, Function);
			case 4: return run_centralized<4, scenario_type>(iter, Function);
			case 5: return run_centralized<5, scenario_type>(iter, Function);
			case 6: return run_centralized<6, scenario_type>(iter, Function);
			// case 7: return run_centralized<7, scenario_type>(iter, Function);
			// case 8: return run_centralized<8, scenario_type>(iter, Function);
			// case 9: return run_centralized<9, scenario_type>(iter, Function);
			// case 10: return run_centralized<10, scenario_type>(iter, Function);
			// case 11: return run_centralized<11, scenario_type>(iter, Function);
			// case 12: return run_centralized<12, scenario_type>(iter, Function);
			// case 13: return run_centralized<13, scenario_type>(iter, Function);
			// case 14: return run_centralized<14, scenario_type>(iter, Function);
			// case 15: return run_centralized<15, scenario_type>(iter, Function);
			// case 16: return run_centralized<16, scenario_type>(iter, Function);
			// case 17: return run_centralized<17, scenario_type>(iter, Function);
			// case 18: return run_centralized<18, scenario_type>(iter, Function);
			// case 19: return run_centralized<19, scenario_type>(iter, Function);
			// case 20: return run_centralized<20, scenario_type>(iter, Function);
			// case 21: return run_centralized<21, scenario_type>(iter, Function);
			// case 22: return run_centralized<22, scenario_type>(iter, Function);
			// case 23: return run_centralized<23, scenario_type>(iter, Function);
			// case 24: return run_centralized<24, scenario_type>(iter, Function);
			// case 25: return run_centralized<25, scenario_type>(iter, Function);
			default: throw MatlabException("Graph Search with '", _config->n_vehicles(), "' vehicles not implemented/available!");
		}
	}

	CentralizedResultStructMex run_centralized(matlab::data::ObjectArray const& iter, std::string const& Function) {
		using namespace GraphBasedPlanning;
		switch (_config->scenario_type()) {
			case SCENARIO_TYPE::Circle: return run_centralized<SCENARIO_TYPE::Circle>(iter, Function);
			case SCENARIO_TYPE::CommonRoad: return run_centralized<SCENARIO_TYPE::CommonRoad>(iter, Function);
			default: throw MatlabException("Scenario type '", _config->scenario_type(), "' not implemented/available!");
		}
	}
};