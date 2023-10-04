#include "../include/ScenarioType.h"

std::ostream& GraphBasedPlanning::operator<<(std::ostream& stream, GraphBasedPlanning::SCENARIO_TYPE scenario_type) {
	switch (scenario_type) {
		case SCENARIO_TYPE::Error: stream << "Error"; break;
		case SCENARIO_TYPE::Circle: stream << "Circle"; break;
		case SCENARIO_TYPE::CommonRoad: stream << "CommonRoad"; break;
	}

	return stream;
}
