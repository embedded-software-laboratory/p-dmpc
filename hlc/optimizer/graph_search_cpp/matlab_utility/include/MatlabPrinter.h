#pragma once

#include "ArrayTypePrinting.h"

// This must follow
#include <Printer.h>

#include <memory>

class MatlabPrinter : public Printer {
	std::shared_ptr<matlab::engine::MATLABEngine>& _matlab;
	matlab::data::ArrayFactory& _factory;

   public:
	static void init(matlab::data::ArrayFactory& _factory, std::shared_ptr<matlab::engine::MATLABEngine>& _matlab);

   private:
	MatlabPrinter(matlab::data::ArrayFactory& _factory, std::shared_ptr<matlab::engine::MATLABEngine>& _matlab) : _matlab(_matlab), _factory(_factory) {}
	void display(std::ostringstream const& stream) final;
};