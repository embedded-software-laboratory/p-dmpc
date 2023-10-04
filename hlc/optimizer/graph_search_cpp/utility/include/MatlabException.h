#pragma once

#include <exception>
#include <sstream>

class MatlabException : public std::exception {
	std::string _exception;

   public:
	template <typename... Args>
	explicit MatlabException(Args &&...args) {
		std::ostringstream ss;

		(ss << ... << args) << std::endl;

		_exception = ss.str();
	}
	[[nodiscard]] char const *what() const noexcept final;
};

class NoSolutionException : public std::exception {
   public:
	[[nodiscard]] char const *what() const noexcept final;
};