#include <ArrayTypePrinting.h>
#include <MatlabPrinter.h>

#include <mex.hpp>
#include <mexAdapter.hpp>

class MexFunction : public matlab::mex::Function {
	std::shared_ptr<matlab::engine::MATLABEngine> _matlab = getEngine();
	matlab::data::ArrayFactory _factory;

   public:
	MexFunction() {
		MatlabPrinter::init(_factory, _matlab);
		Printer::println("MexFunction()");
	}
	void operator()(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) final {
		Printer::println("operator()");
		for (auto const& e : inputs) {
			Printer::print(e.getType(), ", ");
		}
		Printer::println();
	}
	~MexFunction() { Printer::println("~MexFunction()"); }
};