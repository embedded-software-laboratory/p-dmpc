#pragma once

#include <sstream>
#include <stdexcept>

// This class is supposed to be inherited from
class Printer {
	virtual void display(std::ostringstream const& stream) = 0;

   protected:
	static Printer* _instance;
	Printer() = default;

   public:
	Printer(Printer const&) = delete;
	void operator=(Printer const&) = delete;

	template <typename... Args>
	static void println(Args&&... args) {
		if (!_instance) [[unlikely]]
			throw std::logic_error("Printer not initialized!");

		std::ostringstream ss;
		(ss << ... << args) << std::endl;
		_instance->display(ss);
	}

	template <typename... Args>
	static void print(Args&&... args) {
		if (!_instance) [[unlikely]]
			throw std::logic_error("Printer not initialized!");

		std::ostringstream ss;
		(ss << ... << args);
		_instance->display(ss);
	}

	virtual ~Printer();
};