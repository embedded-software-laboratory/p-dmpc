#pragma once

#include <exception>
#include <sstream>

class MexException : public std::exception {
	std::string exception;

   public:
	template <typename... Args>
	explicit MexException(Args &&...args) {
		std::ostringstream ss;

		(ss << ... << args) << std::endl;

		exception = ss.str();
	}
	[[nodiscard]] char const *what() const noexcept final { return exception.c_str(); }
};