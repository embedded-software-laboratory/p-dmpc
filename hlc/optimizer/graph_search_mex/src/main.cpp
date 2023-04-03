#define _USE_MATH_DEFINES
#include <sstream>

#include "mex.hpp"
#include "mexAdapter.hpp"

std::ostream& operator<<(std::ostream& stream, matlab::data::ArrayType const& arrayType) {
	switch (arrayType) {
		case matlab::data::ArrayType::LOGICAL:
			stream << "LOGICAL";
			break;
		case matlab::data::ArrayType::CHAR:
			stream << "CHAR";
			break;
		case matlab::data::ArrayType::MATLAB_STRING:
			stream << "MATLAB_STRING";
			break;
		case matlab::data::ArrayType::DOUBLE:
			stream << "DOUBLE";
			break;
		case matlab::data::ArrayType::SINGLE:
			stream << "SINGLE";
			break;
		case matlab::data::ArrayType::INT8:
			stream << "INT8";
			break;
		case matlab::data::ArrayType::UINT8:
			stream << "UINT8";
			break;
		case matlab::data::ArrayType::INT16:
			stream << "INT16";
			break;
		case matlab::data::ArrayType::UINT16:
			stream << "UINT16";
			break;
		case matlab::data::ArrayType::INT32:
			stream << "INT32";
			break;
		case matlab::data::ArrayType::UINT32:
			stream << "UINT32";
			break;
		case matlab::data::ArrayType::INT64:
			stream << "INT64";
			break;
		case matlab::data::ArrayType::UINT64:
			stream << "UINT64";
			break;
		case matlab::data::ArrayType::COMPLEX_DOUBLE:
			stream << "COMPLEX_DOUBLE";
			break;
		case matlab::data::ArrayType::COMPLEX_SINGLE:
			stream << "COMPLEX_SINGLE";
			break;
		case matlab::data::ArrayType::COMPLEX_INT8:
			stream << "COMPLEX_INT8";
			break;
		case matlab::data::ArrayType::COMPLEX_UINT8:
			stream << "COMPLEX_UINT8";
			break;
		case matlab::data::ArrayType::COMPLEX_INT16:
			stream << "COMPLEX_INT16";
			break;
		case matlab::data::ArrayType::COMPLEX_UINT16:
			stream << "COMPLEX_UINT16";
			break;
		case matlab::data::ArrayType::COMPLEX_INT32:
			stream << "COMPLEX_INT32";
			break;
		case matlab::data::ArrayType::COMPLEX_UINT32:
			stream << "COMPLEX_UINT32";
			break;
		case matlab::data::ArrayType::COMPLEX_INT64:
			stream << "COMPLEX_INT64";
			break;
		case matlab::data::ArrayType::COMPLEX_UINT64:
			stream << "COMPLEX_UINT64";
			break;
		case matlab::data::ArrayType::CELL:
			stream << "CELL";
			break;
		case matlab::data::ArrayType::STRUCT:
			stream << "STRUCT";
			break;
		case matlab::data::ArrayType::OBJECT:
			stream << "OBJECT";
			break;
		case matlab::data::ArrayType::VALUE_OBJECT:
			stream << "VALUE_OBJECT";
			break;
		case matlab::data::ArrayType::HANDLE_OBJECT_REF:
			stream << "HANDLE_OBJECT_REF";
			break;
		case matlab::data::ArrayType::ENUM:
			stream << "ENUM";
			break;
		case matlab::data::ArrayType::SPARSE_LOGICAL:
			stream << "SPARSE_LOGICAL";
			break;
		case matlab::data::ArrayType::SPARSE_DOUBLE:
			stream << "SPARSE_DOUBLE";
			break;
		case matlab::data::ArrayType::SPARSE_COMPLEX_DOUBLE:
			stream << "SPARSE_COMPLEX_DOUBLE";
			break;
		case matlab::data::ArrayType::UNKNOWN:
			stream << "UNKNOWN";
			break;
		default:
			throw "ArrayType has impossible State!";
	}

	return stream;
}

#include <tuple>

#include "GraphSearchCentralizedNaiveMonteCarlo.h"
#include "GraphSearchCentralizedNaiveMonteCarloPolymorphic.h"
#include "GraphSearchCentralizedOptimal.h"
#include "GraphSearchPBOptimal.h"
#include "GraphSearchCentralizedOptimalMemorySaving.h"
#include "GraphSearchCentralizedOptimalPolymorphic.h"
#include "GraphSearchCentralizedOptimalPolymorphicSpeedHeuristic.h"
#include "GraphSearchCentralizedParallelNaiveMonteCarlo.h"
#include "GraphSearchCentralizedParallelNaiveMonteCarloPolymorphic.h"
#include "GraphSearchPBIncrementalOptimal.h"
#include "GraphSearchMEX.h"
#include "MexException.h"

class MatlabPrinter : public Printer {
	std::shared_ptr<matlab::engine::MATLABEngine>& _matlab;
	matlab::data::ArrayFactory& _factory;

   public:
	static void init(matlab::data::ArrayFactory& _factory, std::shared_ptr<matlab::engine::MATLABEngine>& _matlab) {
		if (!_instance) {
			_instance = new MatlabPrinter(_factory, _matlab);
		}
	}

	static void deinit() { delete _instance; }

   private:
	MatlabPrinter(matlab::data::ArrayFactory& _factory, std::shared_ptr<matlab::engine::MATLABEngine>& _matlab) : _matlab(_matlab), _factory(_factory) {}

	void display(std::ostringstream const& stream) final {
		// Pass stream content to MATLAB fprintf function
		_matlab->feval(u"fprintf", 0, std::vector<matlab::data::Array>({_factory.createScalar(stream.str())}));
	}
};

class MexFunction : public matlab::mex::Function,
                    private GraphBasedPlanning::GraphSearchMEX,
                    private GraphBasedPlanning::GraphSearchCentralizedOptimal,
                    private GraphBasedPlanning::GraphSearchCentralizedOptimalMemorySaving,
                    private GraphBasedPlanning::GraphSearchCentralizedOptimalPolymorphic,
                    // private GraphBasedPlanning::GraphSearchCentralizedOptimalPolymorphicSpeedHeuristic,
                    private GraphBasedPlanning::GraphSearchCentralizedNaiveMonteCarlo,
                    private GraphBasedPlanning::GraphSearchCentralizedParallelNaiveMonteCarlo,
                    private GraphBasedPlanning::GraphSearchCentralizedNaiveMonteCarloPolymorphic,
                    private GraphBasedPlanning::GraphSearchCentralizedParallelNaiveMonteCarloPolymorphic,
                    private GraphBasedPlanning::GraphSearchPBOptimal,
                    public GraphBasedPlanning::GraphSearchPBIncrementalOptimal {
	std::shared_ptr<matlab::engine::MATLABEngine> _matlab = getEngine();
	matlab::data::ArrayFactory _factory;

   private:
	// Scenario Data and virtual accessors:
	unsigned int _n_trims = 0U;
	unsigned int _n_hp = 0U;
	unsigned int _n_vehicles = 0U;
	unsigned int _tick_per_step = 0U;
	bool _is_pb = false;
	double _dt = 0.0;
	bool _recursive_feasibility = true;
	GraphBasedPlanning::SCENARIO_TYPE _scenario_type = GraphBasedPlanning::SCENARIO_TYPE::Error;
	std::vector<std::vector<GraphBasedPlanning::CollisionDetection::vec2>> _reference_trajectory;

	[[nodiscard]] inline unsigned int& n_trims() final { return _n_trims; }
	[[nodiscard]] inline unsigned int n_trims() const final { return _n_trims; }

	[[nodiscard]] inline unsigned int& n_hp() final { return _n_hp; }
	[[nodiscard]] inline unsigned int n_hp() const final { return _n_hp; }

	[[nodiscard]] inline unsigned int& n_vehicles() final { return _n_vehicles; }
	[[nodiscard]] inline unsigned int n_vehicles() const final { return _n_vehicles; }

	[[nodiscard]] inline unsigned int& tick_per_step() final { return _tick_per_step; }
	[[nodiscard]] inline unsigned int tick_per_step() const final { return _tick_per_step; }

	[[nodiscard]] inline double& dt() final { return _dt; }
	[[nodiscard]] inline double dt() const final { return _dt; }

	[[nodiscard]] inline bool& is_pb() final { return _is_pb;}
    [[nodiscard]] inline bool is_pb() const final { return _is_pb;}

	[[nodiscard]] inline GraphBasedPlanning::SCENARIO_TYPE& scenario_type() final { return _scenario_type; }
	[[nodiscard]] inline GraphBasedPlanning::SCENARIO_TYPE scenario_type() const final { return _scenario_type; }

	[[nodiscard]] inline std::vector<std::vector<GraphBasedPlanning::CollisionDetection::vec2>>& reference_trajectory() final { return _reference_trajectory; }
	[[nodiscard]] inline std::vector<std::vector<GraphBasedPlanning::CollisionDetection::vec2>> const& reference_trajectory() const final { return _reference_trajectory; }
	
	[[nodiscard]] inline bool& recursive_feasibility() final { return _recursive_feasibility; }
	[[nodiscard]] inline bool recursive_feasibility() const final { return _recursive_feasibility; }

   public:
	MexFunction() : GraphBasedPlanning::GraphSearchMEX(_factory, _matlab) { MatlabPrinter::init(_factory, _matlab); }
	~MexFunction() { MatlabPrinter::deinit(); }

	void operator()(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) final {
		if (inputs.empty()) throw MexException("Wrong number of arguments!");
		if (inputs[0].getType() != matlab::data::ArrayType::ENUM) throw MexException("First Argument must be Enum! (is ", inputs[0].getType(), ")");
		matlab::data::EnumArray FunctionArray = std::move(inputs[0]);
		if (FunctionArray.getClassName() != "Function") throw MexException("First Argument must be Enum of type Function! (is ", FunctionArray.getClassName(), ")");
		std::string Function = FunctionArray[0][0];

		if (Function == "CheckMexFunction") {
			Printer::println("Checked mex Function!");
			matlab::data::TypedArray<bool> out = _factory.createArray({1, 1}, {true});
			outputs[0] = std::move(out);
		} else if (Function == "InitializeWithScenario") {
			if (inputs.size() != 2) throw MexException("Wrong number of arguments! (Must be 2, is ", inputs.size(), ")");
			if (inputs[1].getType() != matlab::data::ArrayType::VALUE_OBJECT) throw MexException("Data must be VALUE_OBJECT! (is ", inputs[1].getType(), ")");

			scenario_callback(inputs[1]);
			Printer::println("C++ Graph Search Initialized");
		} else if (Function == "GraphSearchCentralizedOptimal") {
			if (inputs.size() != 2) throw MexException("Wrong number of arguments! (Must be 2, is ", inputs.size(), ")");
			if (inputs[1].getType() != matlab::data::ArrayType::VALUE_OBJECT) throw MexException("Data must be VALUE_OBJECT! (is ", inputs[1].getType(), ")");

			auto [next_nodes_array, predicted_trims_array, y_predicted_array, n_expanded_array, is_exhausted] = graph_search_callback_centralized(
			    inputs[1], [this]<typename... T>(T... args) { return GraphBasedPlanning::GraphSearchCentralizedOptimal::find_solution(std::forward<T>(args)...); },
			    [this] { return GraphBasedPlanning::GraphSearchCentralizedOptimal::get_n_expanded(); });

			GraphBasedPlanning::GraphSearchCentralizedOptimal::clean();

			outputs[0] = std::move(next_nodes_array);
			outputs[1] = std::move(predicted_trims_array);
			outputs[2] = std::move(y_predicted_array);
			outputs[3] = std::move(n_expanded_array);
			outputs[4] = std::move(is_exhausted);
		} else if (Function == "GraphSearchCentralizedOptimalMemorySaving") {
			if (inputs.size() != 2) throw MexException("Wrong number of arguments! (Must be 2, is ", inputs.size(), ")");
			if (inputs[1].getType() != matlab::data::ArrayType::VALUE_OBJECT) throw MexException("Data must be VALUE_OBJECT! (is ", inputs[1].getType(), ")");

			auto [next_nodes_array, predicted_trims_array, y_predicted_array, n_expanded_array, is_exhausted] = graph_search_callback_centralized(
			    inputs[1], [this]<typename... T>(T... args) { return GraphBasedPlanning::GraphSearchCentralizedOptimalMemorySaving::find_solution(std::forward<T>(args)...); },
			    [this] { return GraphBasedPlanning::GraphSearchCentralizedOptimalMemorySaving::get_n_expanded(); });

			GraphBasedPlanning::GraphSearchCentralizedOptimalMemorySaving::clean();

			outputs[0] = std::move(next_nodes_array);
			outputs[1] = std::move(predicted_trims_array);
			outputs[2] = std::move(y_predicted_array);
			outputs[3] = std::move(n_expanded_array);
			outputs[4] = std::move(is_exhausted);
		} else if (Function == "GraphSearchCentralizedOptimalPolymorphic") {
			if (inputs.size() != 2) throw MexException("Wrong number of arguments! (Must be 2, is ", inputs.size(), ")");
			if (inputs[1].getType() != matlab::data::ArrayType::VALUE_OBJECT) throw MexException("Data must be VALUE_OBJECT! (is ", inputs[1].getType(), ")");

			auto [next_nodes_array, predicted_trims_array, y_predicted_array, n_expanded_array, is_exhausted] = graph_search_callback_centralized(
			    inputs[1], [this]<typename... T>(T... args) { return GraphBasedPlanning::GraphSearchCentralizedOptimalPolymorphic::find_solution(std::forward<T>(args)...); },
			    [this] { return GraphBasedPlanning::GraphSearchCentralizedOptimalPolymorphic::get_n_expanded(); });

			GraphBasedPlanning::GraphSearchCentralizedOptimalPolymorphic::clean();

			outputs[0] = std::move(next_nodes_array);
			outputs[1] = std::move(predicted_trims_array);
			outputs[2] = std::move(y_predicted_array);
			outputs[3] = std::move(n_expanded_array);
			outputs[4] = std::move(is_exhausted);
			/*} else if (Function == "GraphSearchCentralizedOptimalPolymorphicSpeedHeuristic") {
			    if (inputs.size() != 2) throw MexException("Wrong number of arguments! (Must be 2, is ", inputs.size(), ")");
			    if (inputs[1].getType() != matlab::data::ArrayType::VALUE_OBJECT) throw MexException("Data must be VALUE_OBJECT! (is ", inputs[1].getType(), ")");

			    auto [next_nodes_array, predicted_trims_array, y_predicted_array] =
			        graph_search_callback(inputs[1], [this]<typename... T>(T... args) { return GraphBasedPlanning::GraphSearchCentralizedOptimalPolymorphicSpeedHeuristic::find_solution(std::forward<T>(args)...); });

			    GraphBasedPlanning::GraphSearchCentralizedOptimalPolymorphicSpeedHeuristic::clean();

			    outputs[0] = std::move(next_nodes_array);
			    outputs[1] = std::move(predicted_trims_array);
			    outputs[2] = std::move(y_predicted_array);*/
		} else if (Function == "GraphSearchCentralizedNaiveMonteCarlo") {
			if (inputs.size() != 2) throw MexException("Wrong number of arguments! (Must be 2, is ", inputs.size(), ")");
			if (inputs[1].getType() != matlab::data::ArrayType::VALUE_OBJECT) throw MexException("Data must be VALUE_OBJECT! (is ", inputs[1].getType(), ")");

			auto [next_nodes_array, predicted_trims_array, y_predicted_array, n_expanded_array, is_exhausted] = graph_search_callback_centralized(
			    inputs[1], [this]<typename... T>(T... args) { return GraphBasedPlanning::GraphSearchCentralizedNaiveMonteCarlo::find_solution(std::forward<T>(args)...); },
			    [this] { return GraphBasedPlanning::GraphSearchCentralizedNaiveMonteCarlo::get_n_expanded(); });

			GraphBasedPlanning::GraphSearchCentralizedNaiveMonteCarlo::clean();

			outputs[0] = std::move(next_nodes_array);
			outputs[1] = std::move(predicted_trims_array);
			outputs[2] = std::move(y_predicted_array);
			outputs[3] = std::move(n_expanded_array);
			outputs[4] = std::move(is_exhausted);
		} else if (Function == "GraphSearchCentralizedParallelNaiveMonteCarlo") {
			if (inputs.size() != 2) throw MexException("Wrong number of arguments! (Must be 2, is ", inputs.size(), ")");
			if (inputs[1].getType() != matlab::data::ArrayType::VALUE_OBJECT) throw MexException("Data must be VALUE_OBJECT! (is ", inputs[1].getType(), ")");

			auto [next_nodes_array, predicted_trims_array, y_predicted_array, n_expanded_array, is_exhausted] = graph_search_callback_centralized(
			    inputs[1], [this]<typename... T>(T... args) { return GraphBasedPlanning::GraphSearchCentralizedParallelNaiveMonteCarlo::find_solution(std::forward<T>(args)...); },
			    [this] { return GraphBasedPlanning::GraphSearchCentralizedParallelNaiveMonteCarlo::get_n_expanded(); });

			GraphBasedPlanning::GraphSearchCentralizedParallelNaiveMonteCarlo::clean();

			outputs[0] = std::move(next_nodes_array);
			outputs[1] = std::move(predicted_trims_array);
			outputs[2] = std::move(y_predicted_array);
			outputs[3] = std::move(n_expanded_array);
			outputs[4] = std::move(is_exhausted);
		} else if (Function == "GraphSearchCentralizedNaiveMonteCarloPolymorphic") {
			if (inputs.size() != 2) throw MexException("Wrong number of arguments! (Must be 2, is ", inputs.size(), ")");
			if (inputs[1].getType() != matlab::data::ArrayType::VALUE_OBJECT) throw MexException("Data must be VALUE_OBJECT! (is ", inputs[1].getType(), ")");

			auto [next_nodes_array, predicted_trims_array, y_predicted_array, n_expanded_array, is_exhausted] = graph_search_callback_centralized(
			    inputs[1], [this]<typename... T>(T... args) { return GraphBasedPlanning::GraphSearchCentralizedNaiveMonteCarloPolymorphic::find_solution(std::forward<T>(args)...); },
			    [this] { return GraphBasedPlanning::GraphSearchCentralizedNaiveMonteCarloPolymorphic::get_n_expanded(); });

			GraphBasedPlanning::GraphSearchCentralizedNaiveMonteCarloPolymorphic::clean();

			outputs[0] = std::move(next_nodes_array);
			outputs[1] = std::move(predicted_trims_array);
			outputs[2] = std::move(y_predicted_array);
			outputs[3] = std::move(n_expanded_array);
			outputs[4] = std::move(is_exhausted);
		} else if (Function == "GraphSearchCentralizedParallelNaiveMonteCarloPolymorphic") {
			if (inputs.size() != 2) throw MexException("Wrong number of arguments! (Must be 2, is ", inputs.size(), ")");
			if (inputs[1].getType() != matlab::data::ArrayType::VALUE_OBJECT) throw MexException("Data must be VALUE_OBJECT! (is ", inputs[1].getType(), ")");

			auto [next_nodes_array, predicted_trims_array, y_predicted_array, n_expanded_array, is_exhausted] = graph_search_callback_centralized(
			    inputs[1], [this]<typename... T>(T... args) { return GraphBasedPlanning::GraphSearchCentralizedParallelNaiveMonteCarloPolymorphic::find_solution(std::forward<T>(args)...); },
			    [this] { return GraphBasedPlanning::GraphSearchCentralizedParallelNaiveMonteCarloPolymorphic::get_n_expanded(); });

			GraphBasedPlanning::GraphSearchCentralizedParallelNaiveMonteCarloPolymorphic::clean();

			outputs[0] = std::move(next_nodes_array);
			outputs[1] = std::move(predicted_trims_array);
			outputs[2] = std::move(y_predicted_array);
			outputs[3] = std::move(n_expanded_array);
			outputs[4] = std::move(is_exhausted);
		} else if (Function == "GraphSearchPBOptimal") {
			if (inputs.size() != 2) throw MexException("Wrong number of arguments! (Must be 2, is ", inputs.size(), ")");
			if (inputs[1].getType() != matlab::data::ArrayType::VALUE_OBJECT) throw MexException("Data must be VALUE_OBJECT! (is ", inputs[1].getType(), ")");

			auto [next_nodes_array, predicted_trims_array, y_predicted_array, shapes_array, n_expanded_array, is_exhausted] = graph_search_callback_pb(
			    inputs[1], [this]<typename... T>(T... args) { return GraphBasedPlanning::GraphSearchPBOptimal::find_solution(std::forward<T>(args)...); },
			    [this] { return GraphBasedPlanning::GraphSearchPBOptimal::get_n_expanded(); });

			GraphBasedPlanning::GraphSearchPBOptimal::clean();

			outputs[0] = std::move(next_nodes_array);
			outputs[1] = std::move(predicted_trims_array);
			outputs[2] = std::move(y_predicted_array);
			outputs[3] = std::move(shapes_array);
			outputs[4] = std::move(n_expanded_array);
			outputs[5] = std::move(is_exhausted);
		} 
		else if (Function == "GraphSearchPBIncrementalOptimal") {
			if (inputs.size() != 2) throw MexException("Wrong number of arguments! (Must be 2, is ", inputs.size(), ")");
			if (inputs[1].getType() != matlab::data::ArrayType::VALUE_OBJECT) throw MexException("Data must be VALUE_OBJECT! (is ", inputs[1].getType(), ")");

			while (GraphBasedPlanning::GraphSearchPBIncrementalOptimal::preperation_done.load() == false)
				;
			while (GraphBasedPlanning::GraphSearchPBIncrementalOptimal::prep_thread_lock.test_and_set(std::memory_order_acquire))
				;
			GraphBasedPlanning::GraphSearchPBIncrementalOptimal::preperation_done.store(false);

			auto [next_nodes_array, predicted_trims_array, y_predicted_array, shapes_array, n_expanded_array, is_exhausted] = graph_search_callback_pb(
			    std::move(inputs[1]), [this]<typename... T>(T... args) { return GraphBasedPlanning::GraphSearchPBIncrementalOptimal::find_solution(std::forward<T>(args)...); },
			    [this] { return GraphBasedPlanning::GraphSearchPBIncrementalOptimal::get_n_expanded(); });

			outputs[0] = std::move(next_nodes_array);
			outputs[1] = std::move(predicted_trims_array);
			outputs[2] = std::move(y_predicted_array);
			outputs[3] = std::move(shapes_array);
			outputs[4] = std::move(n_expanded_array);
			outputs[5] = std::move(is_exhausted);

			// prepare for next step in extra thread

			GraphBasedPlanning::GraphSearchPBIncrementalOptimal::prep_thread_lock.clear(std::memory_order_release);
			std::thread prep_thread(&GraphBasedPlanning::GraphSearchPBIncrementalOptimal::prepare_next_iteration, this);
			prep_thread.detach();
		} else {
			throw MexException("Function '", Function, "' not implemented/available!");
		}
	}
};