#include <MatlabException.h>

#include <iostream>

int main() {
	try {
		throw MatlabException("hi", 2, 36.9);
	} catch (std::exception& e) {
		std::cout << e.what() << std::endl;
	}
}