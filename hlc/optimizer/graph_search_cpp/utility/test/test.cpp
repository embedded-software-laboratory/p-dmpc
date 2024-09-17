#include <Printer.h>

#include <iostream>

class CoutPrinter : public Printer {
	void display(std::ostringstream const& stream) { std::cout << stream.str() << std::endl; }

	CoutPrinter() = default;

   public:
	static void init() {
		if (_instance) delete _instance;
		_instance = new CoutPrinter();
	}
};

int main() {
	CoutPrinter::init();

	Printer::println(2, "hi", 62.2);
}