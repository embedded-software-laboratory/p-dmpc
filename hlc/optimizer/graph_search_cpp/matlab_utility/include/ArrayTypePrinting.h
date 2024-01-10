#pragma once

#include <ostream>

#include <MatlabDataArray.hpp>
#include <mex.hpp>

std::ostream& operator<<(std::ostream&, matlab::data::ArrayType const&);