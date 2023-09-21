#include "../include/MatlabException.h"

char const* MatlabException::what() const noexcept { return _exception.c_str(); }
char const* NoSolutionException::what() const noexcept { return exception::what(); }
