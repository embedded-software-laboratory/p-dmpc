#include "../include/MatlabPrinter.h"

#include <vector>

void MatlabPrinter::init(matlab::data::ArrayFactory& _factory, std::shared_ptr<matlab::engine::MATLABEngine>& _matlab) {
	if (_instance) delete _instance;
	_instance = new MatlabPrinter(_factory, _matlab);
}
inline void MatlabPrinter::display(const std::ostringstream& stream) {
	// Pass stream content to MATLAB fprintf function
	_matlab->feval(u"fprintf", 0, std::vector<matlab::data::Array>({_factory.createScalar(stream.str())}));
}
