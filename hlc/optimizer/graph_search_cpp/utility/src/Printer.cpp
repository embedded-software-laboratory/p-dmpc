#include "../include/Printer.h"

Printer* Printer::_instance = nullptr;
Printer::~Printer() {
	delete _instance;
	_instance = nullptr;
}
