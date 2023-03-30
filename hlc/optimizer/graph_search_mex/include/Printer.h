#pragma once
#include <sstream>
#include <stdexcept>

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

	virtual ~Printer() = default;
};
Printer* Printer::_instance = nullptr;

/*
namespace GraphBasedPlanning {
    class StringPrinter {
       public:
        template <typename... Args>
        static std::string println(Args &&...args) {
            std::ostringstream ss;

            (ss << ... << args) << std::endl;

            return ss.str();
        }

        template <typename... Args>
        static std::string print(Args &&...args) {
            std::ostringstream ss;

            (ss << ... << args);

            return ss.str();
        }
    };

    class Printer {
        virtual void display(std::ostringstream const &stream) = 0;

       public:
        template <typename... Args>
        void println(Args &&...args) {
            std::ostringstream ss;

            (ss << ... << args) << std::endl;

            display(ss);
        }

        template <typename... Args>
        void print(Args &&...args) {
            std::ostringstream ss;

            (ss << ... << args);

            display(ss);
        }
    };
}  // namespace GraphBasedPlanning*/