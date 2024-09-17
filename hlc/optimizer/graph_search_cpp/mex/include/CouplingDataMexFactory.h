#include <ConfigData.h>
#include <CouplingData.h>

#include <MatlabDataArray.hpp>
#include <mex.hpp>
namespace GraphBasedPlanning {
	CouplingData make_coupling_data(matlab::data::ObjectArray const& iter, std::shared_ptr<ConfigData const> const& config, std::shared_ptr<matlab::engine::MATLABEngine>& _matlab) {
		CouplingData ret = std::vector<std::vector<bool>>(config->n_vehicles(), std::vector<bool>(config->n_vehicles(), false));

		matlab::data::TypedArray<double> const adjacency_array = _matlab->getProperty(iter, u"adjacency");

		for (auto i = 0; i < config->n_vehicles(); ++i) {
			for (auto j = 0; j < config->n_vehicles(); ++j) {
				ret[i][j] = static_cast<bool>(adjacency_array[i][j]);
			}
		}

		return ret;
	}
}  // namespace GraphBasedPlanning
