#ifdef _WIN32
#include <windows.h>
#endif
#pragma once

#include <chrono>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <sstream>

#include "Watchdog.h"

#ifdef __linux__
#include <stdio.h>
#include <sys/resource.h>
#elif __APPLE__
#include <sys/resource.h>
#include <sys/time.h>
#elif _WIN32
#include <Psapi.h>
#endif

#if DO_EVAL
    #define save(saving, ...) EvaluationDataSaver::Instance().saving(__VA_ARGS__)
#endif

using namespace std::chrono_literals;

class EvaluationDataSaver {
	using json = nlohmann::json;

	json _evaluation_data;
	int current_iteration = 0;
	Watchdog dog;

	EvaluationDataSaver() : dog(5s, [this] { save_to_json(); }) {}

   public:
	static EvaluationDataSaver& Instance() {
		static EvaluationDataSaver instance;

		return instance;
	}

	EvaluationDataSaver(EvaluationDataSaver const&) = delete;
	void operator=(EvaluationDataSaver const&) = delete;

	void start_iteration() {
		dog.stop();
		++current_iteration;
	}
	void end_iteration() { dog.start(); }

	void save_to_json() {
		const auto t1 = std::chrono::system_clock::now();
		std::uint64_t timestamp = std::chrono::duration_cast<std::chrono::seconds>(t1.time_since_epoch()).count();
		std::stringstream ss;

		ss << "Evaluation" << timestamp;

		std::ofstream file(ss.str());
		file << _evaluation_data;
		file.close();
	}

	void current_used_memory() {
#ifdef __linux__
		struct rusage r_usage;
		getrusage(RUSAGE_SELF, &r_usage);

		_evaluation_data[current_iteration]["used_memory"] = static_cast<double>(r_usage.ru_maxrss) / 1048576.0;
#elif __APPLE__
		struct rusage r_usage;
		getrusage(RUSAGE_SELF, &r_usage);

		_evaluation_data[current_iteration]["used_memory"] = static_cast<double>(r_usage.ru_maxrss) / 1048576.0;
#elif _WIN32
		PROCESS_MEMORY_COUNTERS_EX pmc;
		GetProcessMemoryInfo(GetCurrentProcess(), (PROCESS_MEMORY_COUNTERS*)&pmc, sizeof(pmc));

		_evaluation_data[current_iteration]["used_memory"] = static_cast<double>(pmc.PrivateUsage) / 1073741824.0;
#endif
	}
	// void cost_to_come(double cost_to_come) { _evaluation_data[current_iteration]["cost_to_come"] = cost_to_come; }
	void time_used(double milliseconds) { _evaluation_data[current_iteration]["time_used"] = milliseconds; }
	void distance_travelled_along_trajectory(const std::vector<double>& distances) { _evaluation_data[current_iteration]["distance_travelled_along_trajectory"] = distances; }
	void average_distance_travelled_along_trajectory(double distance) { _evaluation_data[current_iteration]["average_distance_travelled_along_trajectory"] = distance; }
	void function_used(std::string const& function) { _evaluation_data[current_iteration]["function_used"] = function; }
	void incremental_preparation(double time) { _evaluation_data[current_iteration]["incremental_preparation"] = time; }
	void f_value(double f) { _evaluation_data[current_iteration]["f"] = f; }
	void g_value(double g) {
		static int ci = -1;
		if (ci == current_iteration) {
			_evaluation_data[current_iteration]["g"] = g + _evaluation_data[current_iteration]["g"].get<double>();
		} else {
			ci = current_iteration;
			_evaluation_data[current_iteration]["g"] = g;
		}
	}
	void h_value(double h) { _evaluation_data[current_iteration]["h"] = h; }
	void positives(std::uint64_t positives) { _evaluation_data[current_iteration]["positives"] = positives; }
	void where_conflict(std::vector<std::uint64_t> const& where_conflict) { _evaluation_data[current_iteration]["where_conflict"] = where_conflict; }
	void expansion_time(double expansion_time) { _evaluation_data[current_iteration]["expansion_time"] = expansion_time; }
	void collision_detection_time(double collision_detection_time) { _evaluation_data[current_iteration]["collision_detection_time"] = collision_detection_time; }
	void tested_for_conflict(std::uint64_t tested_for_conflict) { _evaluation_data[current_iteration]["tested_for_conflict"] = tested_for_conflict; }
	void n_expanded(std::uint64_t n_expanded) { _evaluation_data[current_iteration]["n_expanded"] = n_expanded; }
	void n_created(std::uint64_t n_created) { _evaluation_data[current_iteration]["n_created"] = n_created; }
	void branches(std::uint64_t branches) { _evaluation_data[current_iteration]["branches"] = branches; }
	void branches_feasible(std::uint64_t branches_feasible) { _evaluation_data[current_iteration]["branches_feasible"] = branches_feasible; }
	void time_grouping_conflict(double seconds) { _evaluation_data[current_iteration]["time_grouping_conflict"] = seconds; }
	void time_grouping_coupling(double seconds) { _evaluation_data[current_iteration]["time_grouping_coupling"] = seconds; }

	~EvaluationDataSaver() { save_to_json(); }
};